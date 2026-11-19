#!/usr/bin/env python3
"""Local simulation point inspection, constrained jogging and human confirmation.

Preview resets a simulated pose; it is deliberately not a trajectory claim.
This module never initializes ROS or connects to a real robot.
"""
from __future__ import annotations

import argparse
from datetime import datetime
import hashlib
import json
from pathlib import Path
import xml.etree.ElementTree as ET

import mujoco
import numpy as np

from kuavo_arm_controller import now_iso
from kuavo_manual_calibration import ManualCalibration
from kuavo_planar_collision import ObservedCollisionWorld
from kuavo_planar_task import PlanarTask, atomic_json, load_json
from kuavo_table_vision import FourPointCalibration

ROOT = Path(__file__).resolve().parents[1]
BASE = ROOT / 'outputs/kuavo_sim/acceptance_20261014/baseline28'
DEFAULT_SCENE = ROOT / 'outputs/kuavo_sim/manual_corner_check/scene.xml'
DEFAULT_STATE = ROOT / 'outputs/robot_control/manual_corner_check_01'
SEED_HISTORY = ROOT / 'outputs/robot_control/acceptance_20261014_baseline28_v4/history.jsonl'
POINTS = {
    'A': ([0, 0], 'left', 0), 'B': ([1.2, 0], 'right', 0),
    'C': ([1.2, .6], 'right', 0), 'D': ([0, .6], 'left', 0),
    'R1': ([.2, .08], 'left', -10), 'R2': ([1., .08], 'right', 20),
    'R3': ([1., .35], 'right', 20), 'R4': ([.2, .35], 'left', -10),
}


def prepare(scene, state):
    if scene.exists() or (state / 'latest_state.json').exists():
        raise ValueError('Session exists; use its preview/status commands, or choose new --scene and --state-dir paths')
    corners = load_json(BASE / 'assumed_manual_corners.json')
    plane = FourPointCalibration(corners['corners_robot'], 0)
    xml = ET.parse(BASE / 'scene.xml')
    world = xml.getroot().find('worldbody')
    points = {}
    for name, (xy, arm, yaw) in POINTS.items():
        surface, target = plane.robot_xyz(xy, 0), plane.robot_xyz(xy, .05)
        color = '0.95 0.15 0.1 1' if name in 'ABCD' else '0.1 0.85 0.25 1'
        for suffix, position, size in [('surface', surface, .009), ('hover', target, .006)]:
            ET.SubElement(world, 'geom', name=f'check_{name}_{suffix}', type='sphere',
                          pos=' '.join(map(str, position)), size=str(size),
                          rgba=color, density='0', contype='0', conaffinity='0', group='0')
        points[name] = {'table_xy_m': xy, 'arm': arm, 'palm_yaw_deg': yaw,
                        'surface_world_xyz_m': surface.tolist(),
                        'target_world_xyz_m': target.tolist(), 'clearance_m': .05}
    for side, letter, offset, color in [
        ('left', 'l', [0, -.045, -.193], '1 0 1 1'),
        ('right', 'r', [0, .045, -.193], '0 0.85 1 1')]:
        wrist = xml.getroot().find(f".//body[@name='zarm_{letter}7_link']")
        ET.SubElement(wrist, 'geom', name='check_tcp_' + side, type='sphere',
                      pos=' '.join(map(str, offset)), size='.011', rgba=color,
                      density='0', contype='0', conaffinity='0', group='0')
    scene.parent.mkdir(parents=True, exist_ok=True)
    xml.write(scene, encoding='utf-8', xml_declaration=True)
    # Copy the metadata explicitly: this is a geometric simulation inspection,
    # not a demonstration of unknown corners recovered from camera images.
    atomic_json(scene.parent / 'assumed_manual_corners.json', corners)
    atomic_json(scene.parent / 'point_check_targets.json', {
        'points': points, 'scope': 'known simulation targets for manual inspection',
        'source_scene': str(BASE / 'scene.xml'),
        'source_scene_sha256': hashlib.sha256((BASE / 'scene.xml').read_bytes()).hexdigest(),
    })
    first = next(r for r in map(json.loads, SEED_HISTORY.read_text().splitlines())
                 if r.get('status') == 'executed' and r.get('trajectory_arm14_rad'))
    task = PlanarTask(scene, state)
    task.arm.data.qpos[task.arm.qpos_addresses] = first['trajectory_arm14_rad'][0]
    mujoco.mj_forward(task.arm.model, task.arm.data)
    task.save_state('corner_check_initial_snapshot_not_a_motion')
    atomic_json(state / 'corner_check_session.json', {
        'scene': str(scene), 'scene_sha256': task.fingerprint,
        'initial_arm14_rad': first['trajectory_arm14_rad'][0],
        'scope': 'local simulation only; preview is a reset, jog is a checked path',
    })
    return {'status': 'prepared', 'scene': str(scene), 'state_dir': str(state),
            'points': points, 'operator_confirmed': False}


class InspectionWorld(ObservedCollisionWorld):
    """Known simulator geometry is allowed for inspection, never as RGB input."""
    def __init__(self, task, plane):
        super().__init__(task.scene, plane, {'detections': {}})
        self.actual = mujoco.MjModel.from_xml_path(str(task.scene))
        self.actual_data = mujoco.MjData(self.actual)
        if self.actual.nkey:
            mujoco.mj_resetDataKeyframe(self.actual, self.actual_data, 0)
        self.actual_addresses = task.arm.qpos_addresses
        arms = [i for i in range(self.actual.ngeom) if self.actual.geom_group[i] == 3
                and self.actual.body(self.actual.geom_bodyid[i]).name.startswith('zarm_')]
        obstacles = [i for i in range(self.actual.ngeom) if int(self.actual.geom_contype[i]) & 8]
        self.actual_pairs = np.array([(a, b) for a in arms for b in obstacles], dtype=int)

    def check(self, q, tolerance_m=.002):
        hits = super().check(q, tolerance_m)
        m, d = self.actual, self.actual_data
        d.qpos[self.actual_addresses] = q
        mujoco.mj_kinematics(m, d)
        pairs = self.actual_pairs
        delta = d.geom_xpos[pairs[:, 0]] - d.geom_xpos[pairs[:, 1]]
        radius = m.geom_rbound[pairs[:, 0]] + m.geom_rbound[pairs[:, 1]] + .01
        near = np.flatnonzero(np.einsum('ij,ij->i', delta, delta) <= radius * radius)
        for index in near:
            a, b = map(int, pairs[index])
            distance = mujoco.mj_geomDistance(m, d, a, b, .01, None)
            if distance < -tolerance_m:
                hits.append({'robot_body': m.body(m.geom_bodyid[a]).name,
                             'obstacle': m.geom(b).name,
                             'penetration_mm': -float(distance) * 1000,
                             'source': 'actual scene geometry'})
        return hits


def inspect(task, point, spec):
    mujoco.mj_forward(task.arm.model, task.arm.data)
    pos, _, quat = task.arm._eef(spec['arm'])
    delta = pos - spec['target_world_xyz_m']
    rotation = np.zeros(3)
    mujoco.mju_subQuat(rotation, task.down[spec['arm']], quat)
    hits = task.collision_world.check(task.arm._arm14_rad())
    return {'point': point, 'arm': spec['arm'], 'target_world_xyz_m': spec['target_world_xyz_m'],
            'actual_tcp_world_xyz_m': pos.tolist(), 'error_xyz_mm': (delta * 1000).tolist(),
            'position_error_mm': float(np.linalg.norm(delta) * 1000),
            'orientation_error_deg': float(np.degrees(np.linalg.norm(rotation))),
            'collisions': hits, 'arm14_deg': np.degrees(task.arm._arm14_rad()).tolist(),
            'scope': 'simulated TCP, CAD collision tolerance 2 mm; not actual hand contact'}


def archive_ab(task, points, output):
    """Freeze the actual shared A/B pose; never replace it with older IK seeds."""
    if output.exists():
        raise ValueError('Archive already exists; choose a new --output directory')
    raw = task.arm.store.latest_path.read_bytes()
    saved = json.loads(raw)
    np.testing.assert_allclose(saved['arm14_rad'], task.arm._arm14_rad(), atol=1e-12)
    inspections = {name: inspect(task, name, points[name]) for name in ('A', 'B')}
    for name, result in inspections.items():
        if result['position_error_mm'] > 5 or result['orientation_error_deg'] > 1 or result['collisions']:
            raise ValueError(f'Current combined pose does not pass the A/B endpoint gates: {name}: {result}')
    mujoco.mj_forward(task.arm.model, task.arm.data)
    body = task.arm.model.body('waist_yaw').id
    rotation = task.arm.data.xmat[body].reshape(3, 3)
    origin = task.arm.data.xpos[body]
    q = task.arm._arm14_rad()
    record = {
        'schema_version': 1, 'archived_at': now_iso(),
        'purpose': 'user-requested A/B simulation reference for future real-robot comparison',
        'source_state_file': str(task.arm.store.latest_path),
        'source_state_sha256': hashlib.sha256(raw).hexdigest(),
        'source_recorded_at': saved.get('recorded_at'),
        'scene': str(task.scene), 'scene_sha256': task.fingerprint,
        'robot': 'Kuavo-5.2 biped_s52', 'layout': saved['layout'],
        'arm14_rad': q.tolist(), 'arm14_deg': np.degrees(q).tolist(),
        'left_A_deg': np.degrees(q[:7]).tolist(),
        'right_B_deg': np.degrees(q[7:]).tolist(),
        'full_body_qpos': task.arm.data.qpos.tolist(),
        'base_world_xyz_m': task.arm.data.qpos[:3].tolist(),
        'base_world_quaternion_wxyz': task.arm.data.qpos[3:7].tolist(),
        'tcp_offsets_wrist_local_m': task.tcp_offsets,
        'points': {}, 'path_checked': False, 'real_robot_validated': False,
        'operator_confirmation_file_present': (task.state_dir / 'point_checks.json').is_file(),
        'notes': [
            'This is one current combined pose: left TCP at A and right TCP at B, 50 mm above table.',
            'Older preview A may contain an unrelated colliding right-arm pose; it is provenance only.',
            'Simulator wrist-local TCP offsets and fixed crouch have not been calibrated on the real robot.',
            'Real trials must re-read real feedback and solve with measured table/body/TCP geometry.',
        ],
    }
    for name, result in inspections.items():
        position, quat_xyzw, _ = task.arm._eef(result['arm'])
        world_rotation = task.arm.data.xmat[task.arm.body_ids[result['arm']]].reshape(3, 3)
        waist_quat = np.zeros(4)
        mujoco.mju_mat2Quat(waist_quat, (rotation.T @ world_rotation).ravel())
        record['points'][name] = {
            **result, 'table_xy_m': points[name]['table_xy_m'], 'clearance_m': .05,
            'tcp_waist_xyz_m': (rotation.T @ (position-origin)).tolist(),
            'tcp_world_quaternion_xyzw': quat_xyzw.tolist(),
            'tcp_waist_quaternion_xyzw': np.r_[waist_quat[1:], waist_quat[0]].tolist(),
        }
    output.mkdir(parents=True)
    (output / 'source_latest_state.json').write_bytes(raw)
    for name in ('A', 'B'):
        source = task.state_dir / f'corner_check_{name}.json'
        if source.is_file():
            (output / f'source_preview_{name}.json').write_bytes(source.read_bytes())
    atomic_json(output / 'AB_pose.json', record)
    return {'status': 'archived', 'output': str(output / 'AB_pose.json'),
            'arm14_deg': record['arm14_deg'], 'source_state_unchanged': True,
            'path_checked': False, 'real_robot_validated': False}


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--scene', type=Path, default=DEFAULT_SCENE)
    parser.add_argument('--state-dir', type=Path, default=DEFAULT_STATE)
    sub = parser.add_subparsers(dest='command', required=True)
    sub.add_parser('prepare')
    sub.add_parser('status')
    sub.add_parser('reset', help='reset simulated pose to the initial snapshot; retain human records')
    p = sub.add_parser('archive-ab', help='freeze the current combined A/B simulation pose without moving')
    p.add_argument('--output', type=Path, default=ROOT / 'outputs/robot_control/pose_library' /
                   ('kuavo5_AB_' + datetime.now().strftime('%Y%m%d_%H%M%S')))
    p = sub.add_parser('preview', help='reset to a solved endpoint; no transition is validated')
    p.add_argument('point', choices=POINTS)
    p = sub.add_parser('jog', help='checked Cartesian step from the current pose')
    p.add_argument('--arm', choices=['left', 'right'], required=True)
    p.add_argument('--delta', type=float, nargs=3, required=True)
    p.add_argument('--execute', action='store_true')
    p = sub.add_parser('confirm')
    p.add_argument('point', choices=POINTS)
    p.add_argument('--confirm', action='store_true')
    args = parser.parse_args()
    scene, state = args.scene.resolve(), args.state_dir.resolve()
    if args.command == 'prepare':
        print(json.dumps(prepare(scene, state), ensure_ascii=False, indent=2))
        return
    session = load_json(state / 'corner_check_session.json', {})
    if not session or session['scene_sha256'] != hashlib.sha256(scene.read_bytes()).hexdigest():
        raise ValueError('Prepare this scene/state session first; scene changes need a new session')
    points = load_json(scene.parent / 'point_check_targets.json')['points']
    point = getattr(args, 'point', None)
    yaw_args = {}
    if args.command == 'preview':
        yaw_args['palm_yaw_deg' if points[point]['arm'] == 'left' else 'right_palm_yaw_deg'] = points[point]['palm_yaw_deg']
    elif args.command == 'reset':
        initial_yaws = session.get('initial_palm_yaw_deg', {'left': -10, 'right': 20})
        yaw_args = {'palm_yaw_deg': initial_yaws['left'], 'right_palm_yaw_deg': initial_yaws['right']}
    task = PlanarTask(scene, state, **yaw_args)
    supplied = load_json(scene.parent / 'assumed_manual_corners.json')
    plane = FourPointCalibration(supplied['corners_robot'], 0)
    task.collision_world = InspectionWorld(task, plane)
    selection = load_json(state / 'corner_check_selection.json', {})
    if args.command == 'archive-ab':
        result = archive_ab(task, points, args.output.resolve())
    elif args.command == 'reset':
        initial = session.get('initial_arm14_rad')
        if initial is None:
            initial = next(r['trajectory_arm14_rad'][0]
                           for r in map(json.loads, SEED_HISTORY.read_text().splitlines())
                           if r.get('status') == 'executed' and r.get('trajectory_arm14_rad'))
        task.arm.data.qpos[task.arm.qpos_addresses] = initial
        task.save_state('corner_check_reset_snapshot_not_a_motion')
        atomic_json(state / 'corner_check_selection.json', {'mode': 'initial snapshot; transition unchecked'})
        result = {'status': 'reset_snapshot', 'path_checked': False,
                  'human_records_retained': True}
    elif args.command == 'preview':
        spec = points[point]
        print(f'[preview] Solving {point}; a successful result resets the pose, not a checked motion.', flush=True)
        # Solve the pose without obstacle costs so a colliding endpoint can be
        # shown and reported as invalid instead of concealing it from inspection.
        world = task.collision_world
        task.collision_world = None
        solved = task.solve(spec['target_world_xyz_m'], spec['arm'])
        task.collision_world = world
        if not solved['accepted']:
            result = {'status': 'rejected', 'point': point, 'state_unchanged': True,
                      'reason': 'No accepted fixed-palm-down IK endpoint', 'ik': solved}
        else:
            task.arm.data.qpos[task.arm.qpos_addresses] = solved['q']
            task.save_state('corner_preview:' + point + ':snapshot_path_unchecked')
            result = {'status': 'snapshot_shown', **inspect(task, point, spec),
                      'path_checked': False, 'operator_confirmed': False}
            result['endpoint_collision_free'] = not result['collisions']
            mode = 'snapshot; transition unchecked'
            if result['collisions']:
                worst = max(hit['penetration_mm'] for hit in result['collisions'])
                mode += f'; COLLISION {worst:.1f} mm - INVALID'
            atomic_json(state / 'corner_check_selection.json', {'point': point, 'mode': mode})
        atomic_json(state / ('corner_check_' + point + '.json'), result)
    elif args.command == 'jog':
        delta = np.asarray(args.delta, float)
        if not np.isfinite(delta).all() or not 0 < np.linalg.norm(delta) <= .05:
            raise ValueError('Jog must be finite and greater than zero, up to 5 cm')
        goal = task.arm._eef(args.arm)[0] + delta
        result = task.path(goal, args.arm, 'manual_point_jog', args.execute)
    elif args.command == 'confirm':
        if not args.confirm:
            raise ValueError('Inspect the GUI first, then explicitly pass --confirm')
        if selection.get('point') != point:
            raise ValueError('Preview this point before confirming it')
        result = inspect(task, point, points[point])
        if result['position_error_mm'] > 5 or result['orientation_error_deg'] > 1 or result['collisions']:
            raise ValueError('Point fails 5 mm / 1 degree / collision gates; use status and jog before confirmation')
        manual = ManualCalibration(scene, state)
        if point in 'ABCD':
            record = manual.mark(point, result['actual_tcp_world_xyz_m'], .05, True,
                                 'operator_confirmed_simulated_TCP', arm14=task.arm._arm14_rad())
        else:
            record = manual.mark_reference(point, points[point]['table_xy_m'], result['actual_tcp_world_xyz_m'],
                                           .05, True, 'operator_confirmed_simulated_TCP')
            record['arm14_rad'] = task.arm._arm14_rad().tolist()
            record['arm'] = points[point]['arm']
            atomic_json(manual.path, manual.session)
        checks = load_json(state / 'point_checks.json', {})
        checks[point] = {**result, 'operator_confirmed': True, 'recorded_at': now_iso()}
        atomic_json(state / 'point_checks.json', checks)
        result.update(status='operator_confirmed', operator_confirmed=True)
    else:
        result = {'state': task.save_state('corner_check_status'), 'available_points': list(points),
                  'selection': selection, 'confirmed_points': list(load_json(state / 'point_checks.json', {}))}
        if selection.get('point'):
            result['inspection'] = inspect(task, selection['point'], points[selection['point']])
    print(json.dumps(result, ensure_ascii=False, indent=2))
    if result.get('status') == 'rejected':
        raise SystemExit(2)


if __name__ == '__main__':
    main()
