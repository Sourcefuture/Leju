#!/usr/bin/env python3
"""Create an isolated fixed-crouch FK scene; never command a real robot."""
import argparse,json,os,shutil,xml.etree.ElementTree as ET
from pathlib import Path
os.environ.setdefault('MUJOCO_GL','egl')
import mujoco,numpy as np


def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--source-scene',type=Path,required=True);p.add_argument('--output',type=Path,required=True)
    p.add_argument('--hip-deg',type=float,default=-40);p.add_argument('--knee-deg',type=float,default=80)
    p.add_argument('--ankle-deg',type=float,default=-40)
    p.add_argument('--feet-forward-m',type=float,default=0,help='Adjust fixed feet forward while keeping torso/arm workspace unchanged')
    a=p.parse_args();a.output.mkdir(parents=True,exist_ok=True)
    if a.output.resolve()==a.source_scene.parent.resolve():raise ValueError('Use a separate experiment directory')
    if abs(a.hip_deg+a.knee_deg+a.ankle_deg)>1e-6:raise ValueError('Fixed feet require hip+knee+ankle pitch sum zero')
    m=mujoco.MjModel.from_xml_path(str(a.source_scene.resolve()));d=mujoco.MjData(m)
    mujoco.mj_resetDataKeyframe(m,d,0);d.qpos[:3]=[0,0,.98]
    for side in ['l','r']:
        for joint in [3,4,5]:d.qpos[m.joint(f'leg_{side}{joint}_joint').qposadr[0]]=0
    mujoco.mj_forward(m,d);feet=['leg_'+s+'6_link' for s in ['l','r']]
    original=np.mean([d.body(name).xpos for name in feet],axis=0)
    for side in ['l','r']:
        for joint,deg in [(3,a.hip_deg),(4,a.knee_deg),(5,a.ankle_deg)]:
            ident=m.joint(f'leg_{side}{joint}_joint').id;angle=np.radians(deg)
            if not m.jnt_range[ident,0]<=angle<=m.jnt_range[ident,1]:raise ValueError('Leg joint limit exceeded')
            d.qpos[m.jnt_qposadr[ident]]=angle
    mujoco.mj_forward(m,d);d.qpos[:3]+=original-np.mean([d.body(name).xpos for name in feet],axis=0)
    mujoco.mj_forward(m,d);floor=m.geom('tabletop_floor').id
    contact=[i for i in range(m.ngeom) if m.geom_group[i]==3 and m.body(m.geom_bodyid[i]).name in feet]
    clearance=min(mujoco.mj_geomDistance(m,d,i,floor,1.,None) for i in contact)
    d.qpos[2]-=clearance;mujoco.mj_forward(m,d)
    if a.feet_forward_m:
        from scipy.optimize import least_squares
        for side in ['l','r']:
            body=m.body('leg_'+side+'6_link').id
            target=d.xpos[body].copy()+[a.feet_forward_m,0,0]
            joint_ids=[m.joint(f'leg_{side}{j}_joint').id for j in [3,4,5]]
            addresses=m.jnt_qposadr[joint_ids]
            def residual(values):
                d.qpos[addresses]=[values[0],values[1],-values[0]-values[1]]
                mujoco.mj_forward(m,d)
                return d.xpos[body][[0,2]]-target[[0,2]]
            solution=least_squares(residual,d.qpos[addresses[:2]],
                bounds=(m.jnt_range[joint_ids[:2],0],m.jnt_range[joint_ids[:2],1]),
                xtol=1e-12,ftol=1e-12,gtol=1e-12,max_nfev=100)
            if np.linalg.norm(residual(solution.x))>1e-5:raise ValueError('Requested fixed-foot offset is unreachable')
            if np.any(d.qpos[addresses]<m.jnt_range[joint_ids,0]) or np.any(d.qpos[addresses]>m.jnt_range[joint_ids,1]):
                raise ValueError('Foot adjustment exceeds leg joint limits')
    xml=ET.parse(a.source_scene);xml.getroot().find('keyframe/key').set('qpos',' '.join(map(str,d.qpos)))
    xml.write(a.output/'scene.xml',encoding='utf-8',xml_declaration=True)
    for name in ['scene_info.json','assumed_manual_corners.json','environment_loads.json']:
        file=a.source_scene.parent/name
        if file.is_file():shutil.copyfile(file,a.output/name)
    report={'scope':'Fixed-crouch kinematic reset; no WBC or dynamic stability validation',
            'hip_pitch_deg':a.hip_deg,'knee_deg':a.knee_deg,'ankle_pitch_deg':a.ankle_deg,
            'root_world_xyz_m':d.qpos[:3].tolist(),'feet_world_xyz_m':{name:d.body(name).xpos.tolist() for name in feet},
            'sole_adjustment_m':float(clearance),'feet_forward_m':a.feet_forward_m,
            'actual_leg_pitch_degrees':{side:{str(j):float(np.degrees(d.qpos[m.joint(f'leg_{side}{j}_joint').qposadr[0]])) for j in [3,4,5]} for side in ['l','r']}}
    (a.output/'crouch_assumptions.json').write_text(json.dumps(report,indent=2));print(json.dumps(report,indent=2))


if __name__=='__main__':main()
