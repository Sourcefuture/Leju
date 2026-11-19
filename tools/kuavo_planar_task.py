#!/usr/bin/env python3
"""Operator-confirmed planar calibration and palm-down kinematic tasks, simulation only."""
from __future__ import annotations

import argparse
import copy
import hashlib
import json
import math
import os
from pathlib import Path
import uuid

os.environ.setdefault('MUJOCO_GL', 'egl')
import mujoco
import numpy as np

try:
    from kuavo_mujoco_controller import MuJoCoKuavoArmController
    from kuavo_arm_controller import now_iso
except ModuleNotFoundError:
    from tools.kuavo_mujoco_controller import MuJoCoKuavoArmController
    from tools.kuavo_arm_controller import now_iso

ROOT = Path(__file__).resolve().parents[1]
SCENE = ROOT/'outputs/kuavo_sim/tabletop_calibration/scene.xml'
STATE = ROOT/'outputs/robot_control/kuavo5_tabletop'
DOWN_DEG = [0,0,0,-90,-90,0,0, 0,0,0,-90,90,0,0]
BAG_COLORS = ('white','green','black')
CONTACT_OFFSETS = {'left': [0,-.045,-.193], 'right': [0,.045,-.193]}


class PalmContactController(MuJoCoKuavoArmController):
    """Offset the old 193 mm hand reference by 45 mm along the down palm normal."""
    def __init__(self,scene,state_dir,tcp_offsets=None):
        self.tcp_offsets=tcp_offsets or CONTACT_OFFSETS
        super().__init__(scene,state_dir)
    def _eef(self, side):
        _,xyzw,wxyz=super()._eef(side)
        body=self.body_ids[side]
        position=self.data.xpos[body]+self.data.xmat[body].reshape(3,3)@np.array(self.tcp_offsets[side])
        return position.copy(),xyzw,wxyz


def atomic_json(path, payload):
    path=Path(path);path.parent.mkdir(parents=True,exist_ok=True)
    temporary=path.with_suffix('.tmp')
    temporary.write_text(json.dumps(payload,ensure_ascii=False,indent=2)+'\n')
    temporary.replace(path)


def load_json(path, default=None):
    return json.loads(Path(path).read_text()) if Path(path).is_file() else default


def sample_motion(record, start, fraction):
    """Shared GUI/camera trajectory sampling; Cartesian samples constrain orientation."""
    trajectory=record.get('trajectory_arm14_rad')
    if trajectory:
        values=np.asarray(trajectory,dtype=float)
        times=record.get('trajectory_t_s')
        if times:
            times=np.asarray(times,dtype=float);t=np.clip(fraction,0,1)*times[-1]
            i=min(max(0,int(np.searchsorted(times,t,side='right'))-1),len(values)-2)
            a=(t-times[i])/(times[i+1]-times[i])
        else:
            t=np.clip(fraction,0,1)*(len(values)-1)
            i=min(int(t),len(values)-2);a=t-i
        if record.get('trajectory_profile')=='quintic_stop_at_waypoints':
            a=10*a**3-15*a**4+6*a**5
        return values[i]*(1-a)+values[i+1]*a
    a=fraction*fraction*(3-2*fraction)
    return start*(1-a)+np.asarray(record['target_arm14_rad'])*a


def sample_motion_derivatives(record,fraction):
    """Q, velocity and acceleration of the exact trajectory used by viewers."""
    path=np.asarray(record['trajectory_arm14_rad'],float)
    times=np.asarray(record.get('trajectory_t_s',np.linspace(0,record['actual_duration_s'],len(path))),float)
    t=float(np.clip(fraction,0,1))*times[-1]
    index=min(max(0,int(np.searchsorted(times,t,side='right'))-1),len(path)-2)
    dt=times[index+1]-times[index];a=(t-times[index])/dt;delta=path[index+1]-path[index]
    if record.get('trajectory_profile')=='quintic_stop_at_waypoints':
        s=10*a**3-15*a**4+6*a**5;v=30*a*a*(1-a)**2/dt;acc=(60*a-180*a*a+120*a**3)/(dt*dt)
    else:s=a;v=1/dt;acc=0
    return path[index]+delta*s,delta*v,delta*acc


def apply_objects(model, data, objects, progress=1.0, drop_bag=None):
    """Visual kinematic attachment, not finger/contact grasp physics."""
    if not objects:
        return
    for name,spec in objects.items():
        body=model.body(name).id;parent=int(model.body_parentid[body])
        if spec.get('attached_arm'):
            wrist=model.body('zarm_'+('l' if spec['attached_arm']=='left' else 'r')+'7_link').id
            xyz=data.xpos[wrist]+data.xmat[wrist].reshape(3,3)@np.array(spec.get('tcp_local_m',[0,0,-.193]))
            xyz=xyz+np.asarray(spec.get('offset_world_m',[0,0,-spec.get('height_m',.012)]))
        else:
            xyz=np.asarray(spec['world_xyz_m'])
            if 'drop_from_world_xyz_m' in spec and name==drop_bag:
                old=np.asarray(spec['drop_from_world_xyz_m'])
                xyz=old*(1-progress)+xyz*progress
        model.body_pos[body]=data.xmat[parent].reshape(3,3).T@(xyz-data.xpos[parent])
    mujoco.mj_forward(model,data)


class PlanarTask:
    def __init__(self, scene=SCENE, state_dir=STATE, palm_yaw_deg=None, right_palm_yaw_deg=None,tcp_offsets=None):
        self.scene=Path(scene).resolve();self.state_dir=Path(state_dir).resolve()
        self._sim_info=None
        self.fingerprint=hashlib.sha256(self.scene.read_bytes()).hexdigest()
        saved_offsets=load_json(self.state_dir/'latest_state.json',{}).get('contact_tcp_offsets_local_m')
        self.tcp_offsets=tcp_offsets or saved_offsets or copy.deepcopy(CONTACT_OFFSETS)
        if set(self.tcp_offsets)!=set(['left','right']):raise ValueError('TCP offsets need both left and right')
        for vector in self.tcp_offsets.values():
            if np.asarray(vector).shape!=(3,) or not np.isfinite(vector).all() or np.linalg.norm(vector)>.7:raise ValueError('Each TCP offset must be a finite 3-vector in metres, within 0.7 m of wrist')
        if tcp_offsets and saved_offsets and any(not np.allclose(tcp_offsets[s],saved_offsets[s],atol=1e-9,rtol=0) for s in ['left','right']):
            raise ValueError('TCP calibration changed: use a new state directory and repeat table calibration')
        self.arm=PalmContactController(self.scene,self.state_dir,self.tcp_offsets)
        self.cal_path=self.state_dir/'planar_calibration.json'
        self.cal=load_json(self.cal_path,{'points':{},'scene_sha256':self.fingerprint})
        if self.cal['scene_sha256']!=self.fingerprint:
            raise ValueError('Scene changed: use a new state directory or archive the old planar_calibration.json and recalibrate')
        latest=load_json(self.arm.store.latest_path,{})
        self.left_clear_xy=np.asarray(latest.get('left_clear_table_xy_m',[.08,.25]),dtype=float)
        self.objects=latest.get('task_objects',{})
        self.scale_display_kg=float(latest.get('scale_display_kg',0))
        self.down={}
        self.collision_world=None
        self.grasp_bias_overrides={}
        self.palm_yaw_deg=float(latest.get('fixed_palm_yaw_deg_left',-10) if palm_yaw_deg is None else palm_yaw_deg)
        self.right_palm_yaw_deg=float(latest.get('fixed_palm_yaw_deg_right',20) if right_palm_yaw_deg is None else right_palm_yaw_deg)
        original=self.arm._arm14_rad()
        self.arm.data.qpos[self.arm.qpos_addresses]=np.radians(DOWN_DEG)
        mujoco.mj_forward(self.arm.model,self.arm.data)
        for s in ('left','right'):
            yaw=np.radians(self.palm_yaw_deg if s=='left' else self.right_palm_yaw_deg)
            rotation=np.array([np.cos(yaw/2),0,0,np.sin(yaw/2)])
            quat=np.zeros(4);mujoco.mju_mulQuat(quat,rotation,self.arm._eef(s)[2])
            self.down[s]=quat
        self.arm.data.qpos[self.arm.qpos_addresses]=original
        mujoco.mj_forward(self.arm.model,self.arm.data)
        apply_objects(self.arm.model,self.arm.data,self.objects)

    @property
    def info(self):
        """Lazy simulation metadata, used only by explicit calibration/audit helpers."""
        if self._sim_info is None:self._sim_info=load_json(self.scene.parent/'scene_info.json')
        return self._sim_info

    @info.setter
    def info(self,value):
        self._sim_info=value

    def solve(self, xyz, side, start=None, avoid_limits=False):
        c=self.arm;m=c.model;d=c.data
        original=c._arm14_rad();start=original if start is None else np.asarray(start)
        sl=slice(0,7) if side=='left' else slice(7,14)
        qa=c.qpos_addresses[sl];da=c.dof_addresses[sl];ids=c.joint_ids[sl];body=c.body_ids[side]
        candidates=[start.copy()]
        seed=start.copy();seed[sl]=np.radians(DOWN_DEG)[sl];candidates.append(seed)
        for shoulder,roll,elbow in [(-.6,.5,-1.2),(.4,.8,-2),(-1,.3,-1)]:
            seed=start.copy();seed[sl]=np.radians(DOWN_DEG)[sl]
            seed[sl][0]=shoulder;seed[sl][1]=roll if side=='left' else -roll;seed[sl][3]=elbow
            candidates.append(seed)
        for shoulder,yaw in [(-.4,.7),(-.8,.4),(0.,.9)]:
            seed=start.copy();seed[sl]=np.array([shoulder,-.25 if side=='left' else .25,
                -yaw if side=='left' else yaw,-1.7,-.8 if side=='left' else .8,
                -.3 if side=='left' else .3,.1])
            candidates.append(seed)
        # Elbow-up branch: preserve the same world palm orientation using the
        # opposite wrist-yaw branch, rather than forcing an elbow through the bin.
        for shoulder,roll in [(.3,2.6),(.6,2.9),(0.,2.3)]:
            seed=start.copy();seed[sl]=np.array([shoulder,roll if side=='left' else -roll,
                -.4 if side=='left' else .4,-1.5,1.2 if side=='left' else -1.2,
                -.3 if side=='left' else .3,.1])
            candidates.append(seed)
        def safe(values):
            return self.collision_world is None or not self.collision_world.check(values)
        best=None
        try:
            for seed in candidates:
                d.qpos[c.qpos_addresses]=seed
                last_error=float('inf');stagnant=0
                for iteration in range(300):
                    mujoco.mj_kinematics(m,d);mujoco.mj_comPos(m,d)
                    pos,_,quat=c._eef(side)
                    dp=np.asarray(xyz)-pos;dr=np.zeros(3)
                    mujoco.mju_subQuat(dr,self.down[side],quat)
                    if np.linalg.norm(dp)<.0005 and np.linalg.norm(dr)<math.radians(.2):break
                    # A redundant joint-limit objective can keep moving in
                    # the nullspace after the task pose is already accurate.
                    # Stop safely instead of spending 500 iterations at every
                    # 1 cm waypoint. Final FK/interpolation audits still apply.
                    if iteration>=25 and np.linalg.norm(dp)<.001 and np.linalg.norm(dr)<math.radians(.35) and safe(c._arm14_rad()):break
                    error=float(np.linalg.norm(dp)+np.linalg.norm(dr))
                    stagnant=stagnant+1 if abs(last_error-error)<1e-8 else 0
                    last_error=error
                    if stagnant>=20:break
                    jp=np.zeros((3,m.nv));jr=np.zeros((3,m.nv))
                    mujoco.mj_jac(m,d,jp,jr,pos,body)
                    jac=np.vstack([jp[:,da],jr[:,da]])
                    delta=jac.T@np.linalg.solve(jac@jac.T+1e-4*np.eye(6),np.r_[dp,dr])
                    if avoid_limits:
                        limits=m.jnt_range[ids];half=(limits[:,1]-limits[:,0])/2
                        normalized=(d.qpos[qa]-limits.mean(axis=1))/half
                        gradient=-np.sign(normalized)*np.clip((np.abs(normalized)-.65)/.35,0,1)**2*.04
                        nullspace=np.eye(7)-np.linalg.pinv(jac,rcond=1e-6)@jac
                        delta+=nullspace@gradient
                    delta*=min(1,.06/max(1e-12,np.max(abs(delta))))
                    d.qpos[qa]+=delta
                    for a,j in zip(qa,ids):d.qpos[a]=np.clip(d.qpos[a],*m.jnt_range[j])
                mujoco.mj_kinematics(m,d);mujoco.mj_comPos(m,d)
                pos,_,quat=c._eef(side);dr=np.zeros(3);mujoco.mju_subQuat(dr,self.down[side],quat)
                result={'q':c._arm14_rad().tolist(),'position_error_mm':float(np.linalg.norm(np.asarray(xyz)-pos)*1000),
                        'orientation_error_deg':float(np.degrees(np.linalg.norm(dr))),'achieved_xyz_m':pos.tolist()}
                result['collision_free']=safe(result['q'])
                score=result['position_error_mm']+result['orientation_error_deg']*5+(0 if result['collision_free'] else 1e5)
                if best is None or score<best[0]:best=(score,result)
                if result['position_error_mm']<=3 and result['orientation_error_deg']<=1 and result['collision_free']:break
            result=best[1]
            if result['position_error_mm']>3 or result['orientation_error_deg']>1 or not result['collision_free']:
                from scipy.optimize import least_squares
                bounds=m.jnt_range[ids]
                def residual(values):
                    d.qpos[c.qpos_addresses]=start;d.qpos[qa]=values
                    mujoco.mj_kinematics(m,d);mujoco.mj_comPos(m,d)
                    pos,_,quat=c._eef(side);dr=np.zeros(3)
                    mujoco.mju_subQuat(dr,self.down[side],quat)
                    base=np.r_[np.asarray(xyz)-pos,dr]
                    if self.collision_world is not None:
                        return np.r_[base,5*self.collision_world.penalties(c._arm14_rad(),side)]
                    return base
                for seed in candidates:
                    fitted=least_squares(residual,np.clip(seed[sl],bounds[:,0]+1e-8,bounds[:,1]-1e-8),
                        bounds=(bounds[:,0],bounds[:,1]),max_nfev=250,ftol=1e-8,xtol=1e-8,gtol=1e-8)
                    residual(fitted.x)
                    pos,_,quat=c._eef(side);dr=np.zeros(3);mujoco.mju_subQuat(dr,self.down[side],quat)
                    r={'q':c._arm14_rad().tolist(),'position_error_mm':float(np.linalg.norm(np.asarray(xyz)-pos)*1000),
                        'orientation_error_deg':float(np.degrees(np.linalg.norm(dr))),'achieved_xyz_m':pos.tolist()}
                    r['collision_free']=safe(r['q'])
                    score=r['position_error_mm']+5*r['orientation_error_deg']+(0 if r['collision_free'] else 1e5)
                    if score<best[0]:best=(score,r)
                    if r['position_error_mm']<=3 and r['orientation_error_deg']<=1 and r['collision_free']:break
                result=best[1]
            result['accepted']=result['position_error_mm']<=3 and result['orientation_error_deg']<=1 and result['collision_free']
            return result
        finally:
            d.qpos[c.qpos_addresses]=original;mujoco.mj_kinematics(m,d);mujoco.mj_comPos(m,d)

    def truth_xyz(self, xy, clearance):
        return (np.asarray(self.info['T_world_table'])@np.r_[xy,clearance,1])[:3]

    def initialize(self, observations=None):
        initial=self.arm._arm14_rad().copy()
        reference=np.radians(DOWN_DEG)
        self.arm.data.qpos[self.arm.qpos_addresses]=reference
        mujoco.mj_forward(self.arm.model,self.arm.data)
        positions={side:self.arm._eef(side)[0].copy() for side in ('left','right')}
        # Initialization is a simulated reset into one working posture, not a
        # validated real-world transition from hanging arms through the table.
        setup_z=max(1.25,self.calibrated_xyz([.6,.3],.30)[2]) if self.cal.get('fit') else 1.25
        for side in positions:
            positions[side][0]=.64
            positions[side][2]=setup_z
        if self.cal.get('fit'):
            if observations is not None:
                bags=[det['table_xy_m'][0] for name,det in observations['detections'].items() if name.startswith('bag_')]
                if bags:
                    edge=min(bags)-.15
                    if self.left_clear_xy[0]>=edge:
                        self.left_clear_xy[0]=max(-.06,edge-.05)
            positions['left']=self.calibrated_xyz(self.left_clear_xy,.14)
            park=self.calibrated_xyz([1.22,.25],0)
            park[2]=setup_z;positions['right']=park
            if observations is not None:
                box=np.asarray(observations['detections']['material_box']['table_xy_m'])
                positions['right']=self.calibrated_xyz(box,.285)
        else:
            # A local, reachable preparation pose for manual calibration. The
            # former hard-coded right Y=-.62 m exceeded this crouched workspace.
            root=self.arm.data.body('base_link')
            rotation=root.xmat.reshape(3,3)
            for side in ('left','right'):
                positions[side]=root.xpos+rotation@np.array([.30,.40 if side=='left' else -.40,.40])
        if observations is not None:self.set_collision_world(observations)
        self.arm.data.qpos[self.arm.qpos_addresses]=initial;mujoco.mj_forward(self.arm.model,self.arm.data)
        target=initial.copy()
        for side in ('left','right'):
            if side=='right' and observations is not None:
                # A model-only wrist branch seed. It contains no scene/object
                # location and avoids initializing at the wrist-roll limit.
                target[7:]=[-.4,-.5,-.5,-2.1,1.,1.1,-.5]
            result=self.solve(positions[side],side,target)
            if not result['accepted'] and side=='left' and observations is not None:
                # A source close to the table's left edge can push the old
                # parking heuristic outside the arm workspace. Try bounded
                # empty-hand parking poses from the same manual plane; every
                # candidate still receives the full pose/collision check.
                attempts=[]
                for xy in ([.08,.25],[.18,0],[.25,0],[.35,0],[.18,.10],[.25,.10],[.35,.10]):
                    candidate=self.calibrated_xyz(xy,.14)
                    trial=self.solve(candidate,side,target)
                    attempts.append({'table_xy_m':xy,'accepted':trial['accepted'],
                                     'position_error_mm':trial['position_error_mm']})
                    if trial['accepted']:
                        self.left_clear_xy=np.asarray(xy,float);positions[side]=candidate;result=trial
                        print('[setup] selected reachable left parking point '+str(xy),flush=True)
                        break
                atomic_json(self.state_dir/'setup_parking_search.json',{'attempts':attempts,
                    'selected_table_xy_m':self.left_clear_xy.tolist() if result['accepted'] else None,
                    'scope':'local simulation reset selection from manual plane + RGB obstacles + robot CAD; no real transition validated'})
            if not result['accepted']:raise ValueError('Fixed palm-down setup pose is unreachable: '+json.dumps({'arm':side,'xyz_m':positions[side].tolist(),'ik':result}))
            target=np.asarray(result['q'])
        self.arm.data.qpos[self.arm.qpos_addresses]=target
        mujoco.mj_forward(self.arm.model,self.arm.data)
        record={'status':'initialized'}
        self.save_state('palm_down_initialized')
        return {'status':record['status'],'note':'Working orientation fixed from this palm-down reference; setup transition changes orientation'}

    def save_state(self, reason):
        state=self.arm.read_current_state(reason,save=False)
        state['task_objects']=copy.deepcopy(self.objects)
        state['working_orientation_wxyz']={k:v.tolist() for k,v in self.down.items()}
        state['fixed_palm_yaw_deg_left']=self.palm_yaw_deg
        state['fixed_palm_yaw_deg_right']=self.right_palm_yaw_deg
        state['eef_reference']='configured wrist-local TCP; defaults are the 193 mm hand reference plus 45 mm proxy palm-normal offset, not real fingertip calibration'
        state['contact_tcp_offsets_local_m']=copy.deepcopy(self.tcp_offsets)
        state['scale_display_kg']=self.scale_display_kg
        state['left_clear_table_xy_m']=self.left_clear_xy.tolist()
        state['head_yaw_pitch_deg']=[float(np.degrees(self.arm.data.qpos[self.arm.model.joint(j).qposadr[0]])) for j in ['zhead_1_joint','zhead_2_joint']]
        self.arm.store.write_latest(state)
        return state

    def set_head_command(self,values):
        for name,value in zip(['zhead_1_joint','zhead_2_joint'],values):
            ident=self.arm.model.joint(name).id;angle=np.radians(value)
            if not self.arm.model.jnt_range[ident,0]<=angle<=self.arm.model.jnt_range[ident,1]:raise ValueError('Head view exceeds model joint range')
            self.arm.data.qpos[self.arm.model.jnt_qposadr[ident]]=angle
        mujoco.mj_forward(self.arm.model,self.arm.data)

    def path(self, xyz, side, label, execute=False, duration=2):
        """Validate a direct route, then bounded planar transit detours if needed."""
        c=self.arm;original=c._arm14_rad().copy();start=c._eef(side)[0].copy()
        record=self._path_straight(xyz,side,label,False,duration)
        if record['status']=='rejected' and record.get('reason')=='joint interpolation violates pose/collision constraints':
            for resolution in [.005,.0025]:
                refined=self._path_straight(xyz,side,label,False,duration,cartesian_step_m=resolution)
                if refined['status']=='planned':
                    refined['adaptive_cartesian_step_m']=resolution
                    refined['refinement_reason']='coarse joint interpolation violated orientation/collision constraints'
                    record=refined;break
        if record['status']=='rejected' and self.cal.get('fit') and any(t in label for t in ('transfer','hover','clear','retreat')):
            axis=np.asarray(self.cal['fit']['affine_world_from_xy'])[2]
            first_failure=record
            alternatives=[(axis,v,'table_y') for v in [.025,.05,.075,.10,-.025,-.05]]
            alternatives += [(np.asarray(self.cal['fit']['affine_world_from_xy'])[1],v,'table_x') for v in [.05,-.05]]
            for direction,offset,axis_name in alternatives:
                middle=(start+np.asarray(xyz))/2+direction*offset
                segments=[]
                try:
                    for waypoint in [middle,np.asarray(xyz)]:
                        segment=self._path_straight(waypoint,side,label,False,duration/2,cartesian_step_m=.005)
                        if segment['status']=='rejected':break
                        segments.append(segment)
                        c.data.qpos[c.qpos_addresses]=segment['target_arm14_rad'];mujoco.mj_forward(c.model,c.data)
                finally:
                    c.data.qpos[c.qpos_addresses]=original;mujoco.mj_forward(c.model,c.data)
                if len(segments)!=2:continue
                record=dict(segments[-1]);record['trajectory_arm14_rad']=segments[0]['trajectory_arm14_rad']+segments[1]['trajectory_arm14_rad'][1:]
                elapsed=segments[0]['actual_duration_s']
                record['trajectory_t_s']=segments[0]['trajectory_t_s']+[elapsed+t for t in segments[1]['trajectory_t_s'][1:]]
                record['actual_duration_s']=elapsed+segments[1]['actual_duration_s']
                record['requested_duration_s']=float(duration)
                record['max_position_error_mm']=max(s['max_position_error_mm'] for s in segments)
                record['max_orientation_error_deg']=max(s['max_orientation_error_deg'] for s in segments)
                record['path_adjustments']=sum([s['path_adjustments'] for s in segments],[])
                record['transit_detour']={'waypoint_robot_xyz_m':middle.tolist(),'offset_axis':axis_name,'offset_m':offset,
                                         'table_y_offset_m':offset if axis_name=='table_y' else 0,
                                         'direct_failure_reason':first_failure.get('reason','IK infeasible')}
                break
        if execute and record['status']=='planned':
            record['status']='executed'
            c.data.qpos[c.qpos_addresses]=record['target_arm14_rad'];mujoco.mj_forward(c.model,c.data)
            apply_objects(c.model,c.data,self.objects)
            self.save_state(label);c.store.append_history(record)
        return record

    def _path_straight(self, xyz, side, label, execute=False, duration=2,cartesian_step_m=.01):
        c=self.arm;q=c._arm14_rad();start=c._eef(side)[0]
        # Reject starting a constrained path from a different orientation.
        dr=np.zeros(3);mujoco.mju_subQuat(dr,self.down[side],c._eef(side)[2])
        if np.linalg.norm(dr)>math.radians(1):raise ValueError('Run planar init first; palm orientation is not down')
        steps=max(1,int(np.ceil(np.linalg.norm(np.asarray(xyz)-start)/cartesian_step_m)))
        trajectory=[q.tolist()];max_error=0;max_angle=0;adjustments=[]
        points=np.linspace(start,xyz,steps+1)[1:]
        travel=any(token in label for token in ('hover','up','raise','clear','retreat','transfer'))
        for index,p in enumerate(points):
            r=self.solve(p,side,q,avoid_limits=True)
            if not r['accepted'] and travel and index<len(points)-1 and r.get('collision_free') and r['orientation_error_deg']<=1 and r['position_error_mm']<=15:
                revised=np.asarray(r['achieved_xyz_m'])
                verified=self.solve(revised,side,np.asarray(r['q']),avoid_limits=True)
                if verified['accepted']:
                    adjustments.append({'nominal_xyz_m':p.tolist(),'planned_xyz_m':revised.tolist(),
                                        'offset_mm':r['position_error_mm'],'reason':'reachable collision-free travel detour'})
                    r=verified
            if not r['accepted']:
                return {'status':'rejected','stage':label,'arm':side,'requested_xyz_m':np.asarray(xyz).tolist(),
                        'failed_waypoint_m':p.tolist(),'ik':r}
            q=np.asarray(r['q']);trajectory.append(q.tolist())
            max_error=max(max_error,r['position_error_mm']);max_angle=max(max_angle,r['orientation_error_deg'])
        # Verify joint interpolation, not just Cartesian endpoints.
        original=c._arm14_rad().copy()
        try:
            for index,(before,after) in enumerate(zip(trajectory[:-1],trajectory[1:])):
                for fraction in [.25,.5,.75]:
                    middle=np.array(before)*(1-fraction)+np.array(after)*fraction
                    c.data.qpos[c.qpos_addresses]=middle;mujoco.mj_forward(c.model,c.data)
                    dr=np.zeros(3);mujoco.mju_subQuat(dr,self.down[side],c._eef(side)[2])
                    collisions=self.collision_world.check(middle) if self.collision_world else []
                    if np.linalg.norm(dr)>math.radians(1) or collisions:
                        return {'status':'rejected','stage':label,'reason':'joint interpolation violates pose/collision constraints',
                                'arm':side,'requested_xyz_m':np.asarray(xyz).tolist(),'interpolation_fraction':fraction,
                                'segment_index':index,'orientation_error_deg':float(np.degrees(np.linalg.norm(dr))),
                                'largest_joint_step_deg':float(np.max(np.abs(np.degrees(np.array(after)-before)))),
                                'collisions':collisions,'before_q':before,'after_q':after}
        finally:
            c.data.qpos[c.qpos_addresses]=original;mujoco.mj_forward(c.model,c.data)
        changes=np.abs(np.degrees(np.diff(np.array(trajectory),axis=0))).max(axis=1)
        # Quintic segments stay on the verified joint-interpolation locus and
        # have zero speed/acceleration at boundaries. Bound peak derivatives,
        # not just average speed. This is conservative, with a stop every cm.
        intervals=np.maximum.reduce([np.full(len(changes),float(duration)/(len(trajectory)-1)),
            1.875*changes/25.,np.sqrt(5.773502692*changes/90.),np.cbrt(60*changes/500.)])
        times=np.r_[0,np.cumsum(intervals)]
        safe_duration=float(times[-1])
        record={'schema_version':1,'action_id':str(uuid.uuid4()),'created_at':now_iso(),'kind':'planar_task',
                'name':label,'status':'executed' if execute else 'planned','arm':side,
                'target_arm14_rad':q.tolist(),'target_arm14_deg':np.degrees(q).tolist(),
                'trajectory_arm14_rad':trajectory,'actual_duration_s':safe_duration,
                'trajectory_t_s':times.tolist(),
                'requested_duration_s':float(duration),'joint_speed_bound_deg_s':25.,
                'trajectory_profile':'quintic_stop_at_waypoints','joint_acceleration_bound_deg_s2':90.,
                'joint_jerk_bound_deg_s3':500.,
                'requested_xyz_m':np.asarray(xyz).tolist(),'reference_frame':'MuJoCo world',
                'max_position_error_mm':max_error,'max_orientation_error_deg':max_angle,
                'path_adjustments':adjustments,
                'task_objects':copy.deepcopy(self.objects)}
        record['head_yaw_pitch_deg']=[float(np.degrees(c.data.qpos[c.model.joint(j).qposadr[0]])) for j in ['zhead_1_joint','zhead_2_joint']]
        if execute:
            c.data.qpos[c.qpos_addresses]=q;mujoco.mj_forward(c.model,c.data)
            apply_objects(c.model,c.data,self.objects)
            self.save_state(label);c.store.append_history(record)
        return record

    def approach(self, point, clearance, execute=False, xy=None):
        xy=xy if xy is not None else self.info['calibration_points'][{'A':'A_near_left','B':'B_near_right','C':'C_far_right','D':'D_far_left'}[point]]['table_xyz_m'][:2]
        side='left' if xy[0]<.6 else 'right'
        result=self.path(self.truth_xyz(xy,clearance),side,'calibrate:'+point,execute)
        if execute and result['status']=='executed':
            atomic_json(self.state_dir/'calibration_pending.json',{'point':point,'table_xy_m':xy,
                        'clearance_m':clearance,'arm':side,'action_id':result['action_id']})
        return result

    def record(self, point, confirmed=False):
        if not confirmed:raise ValueError('Human confirmation required: inspect GUI, then add --confirm')
        pending=load_json(self.state_dir/'calibration_pending.json')
        if not pending or pending['point']!=point:raise ValueError('No successful pending approach for this point')
        side=pending['arm'];xyz=self.arm._eef(side)[0]
        expected=self.truth_xyz(pending['table_xy_m'],pending['clearance_m'])
        if np.linalg.norm(xyz-expected)>.01:raise ValueError('Arm changed materially; approach or jog back to the point before recording')
        dr=np.zeros(3);mujoco.mju_subQuat(dr,self.down[side],self.arm._eef(side)[2])
        if np.linalg.norm(dr)>math.radians(1):raise ValueError('Palm orientation changed')
        self.cal['points'][point]={**pending,'measured_world_xyz_m':xyz.tolist(),
            'measured_arm14_rad':self.arm._arm14_rad().tolist(),'operator_confirmed':True,'recorded_at':now_iso()}
        self.cal.pop('fit',None);atomic_json(self.cal_path,self.cal)
        return {'status':'recorded','point':point,'record':self.cal['points'][point]}

    def fit(self):
        if set(self.cal['points'])!={'A','B','C','D'}:raise ValueError('Four human-confirmed points A,B,C,D are required')
        points=list(self.cal['points'].values());clearance=np.array([p['clearance_m'] for p in points])
        if np.ptp(clearance)>.001:raise ValueError('Four points must use the same clearance')
        design=np.array([[1,*p['table_xy_m']] for p in points])
        if np.linalg.matrix_rank(design)!=3:raise ValueError('Calibration points must not be collinear')
        measured=np.array([p['measured_world_xyz_m'] for p in points])
        coeff=np.linalg.lstsq(design,measured,rcond=None)[0]
        errors=np.linalg.norm(design@coeff-measured,axis=1)
        if max(errors)>.005:raise ValueError('Planar fit residual exceeds 5 mm')
        normal=np.cross(coeff[1],coeff[2]);normal/=np.linalg.norm(normal)
        if normal[2]<0:normal=-normal
        if normal[2]<.999:raise ValueError('This fixed-palm-down implementation requires an approximately horizontal table')
        self.cal['fit']={'affine_world_from_xy':coeff.tolist(),'normal_world':normal.tolist(),
            'calibration_clearance_m':float(clearance[0]),'max_residual_mm':float(max(errors)*1000)}
        atomic_json(self.cal_path,self.cal);return self.cal['fit']

    def import_corners(self, path):
        supplied=load_json(path)
        if supplied.get('manual_session_file'):
            session=load_json(supplied['manual_session_file'])
            if not session or session.get('scene_sha256')!=self.fingerprint:
                raise ValueError('Manual calibration session is missing or belongs to a different scene')
            if not session.get('fit'):
                raise ValueError('Manual measurements changed: fit and export the corners again')
            if supplied.get('manual_measurements_sha256'):
                from kuavo_manual_calibration import measurement_digest
                if measurement_digest(session,supplied['manual_measurements_key'])!=supplied['manual_measurements_sha256']:
                    raise ValueError('Manual corner export is stale: fit and export the edited measurements again')
        try:
            from kuavo_table_vision import FourPointCalibration
        except ModuleNotFoundError:
            from tools.kuavo_table_vision import FourPointCalibration
        clearance=float(supplied.get('clearance_m',0))
        corners=supplied['corners_robot']
        frame=supplied.get('reference_frame','mujoco_world')
        if frame in ('waist_yaw','waist_yaw_link'):
            body=self.arm.model.body('waist_yaw').id
            rotation=self.arm.data.xmat[body].reshape(3,3);origin=self.arm.data.xpos[body]
            corners={k:(origin+rotation@np.asarray(v,dtype=float)).tolist() for k,v in corners.items()}
        elif frame not in ('world','mujoco_world'):
            raise ValueError('Corner reference_frame must be mujoco_world or waist_yaw_link')
        FourPointCalibration(corners,clearance)
        self.cal={'scene_sha256':self.fingerprint,'points':{},
                  'source':'externally supplied manual corner XYZ; assumed known for simulation',
                  'input_provenance':supplied.get('provenance','unspecified supplied coordinates'),
                  'manual_session_file':supplied.get('manual_session_file'),
                  'input_reference_frame':frame,'internal_reference_frame':'mujoco_world'}
        for key,xy in zip('ABCD',[[0,0],[1.2,0],[1.2,.6],[0,.6]]):
            self.cal['points'][key]={'table_xy_m':xy,'clearance_m':clearance,
                'measured_world_xyz_m':corners[key],
                'source':'supplied coordinates, not a simulated arm arrival confirmation'}
        return self.fit()

    def calibrated_xyz(self, xy, height):
        f=self.cal.get('fit')
        if not f:raise ValueError('Fit four human-confirmed points before task execution')
        coeff=np.array(f['affine_world_from_xy']);n=np.array(f['normal_world'])
        return np.array([1,*xy])@coeff+n*(height-f['calibration_clearance_m'])

    def set_collision_world(self, observation):
        try:
            from kuavo_planar_collision import ObservedCollisionWorld
            from kuavo_table_vision import FourPointCalibration
        except ModuleNotFoundError:
            from tools.kuavo_planar_collision import ObservedCollisionWorld
            from tools.kuavo_table_vision import FourPointCalibration
        corners={k:p['measured_world_xyz_m'] for k,p in self.cal['points'].items()}
        calibration=FourPointCalibration(corners,self.cal['fit']['calibration_clearance_m'])
        self.collision_world=ObservedCollisionWorld(self.scene,calibration,observation)
        self.collision_world.set_packed_count(sum(o.get('destination')=='bin' for o in self.objects.values()))

    def audit(self,clearance):
        rows=[]
        for name,xy in [('A',[0,0]),('B',[1.2,0]),('C',[1.2,.6]),('D',[0,.6])]:
            side='left' if xy[0]<.6 else 'right'
            r=self.solve(self.truth_xyz(xy,clearance),side)
            rows.append({'point':name,'arm':side,'reachable':r['accepted'],
                         'position_error_mm':r['position_error_mm'],'orientation_error_deg':r['orientation_error_deg']})
        for item in ('bag_white','bag_green','bag_black','scale','material_box'):
            xy=self.info['items'][item]['xy'];height=self.info['items'][item]['size_m'][2]
            options={side:self.solve(self.truth_xyz(xy,height),side) for side in ('left','right')}
            rows.append({'item':item,'height_m':height,'reachable_arms':[s for s,r in options.items() if r['accepted']]})
        atomic_json(self.state_dir/'reachability.json',rows);return rows

    def stages(self, color, observations, stack_index=0):
        """Use calibrated XY mapping and fixed heights for the rigid demo bags."""
        bag='bag_'+color
        source=np.array(observations['detections'][bag]['table_xy_m'])
        scale=np.array(observations['detections']['scale']['table_xy_m'])
        box=np.array(observations['detections']['material_box']['table_xy_m'])
        source_z=.012;scale_z=.02+source_z
        box_z=.23+source_z+.04
        travel=.14
        bin_travel=.285
        scale_transfer_height=.26
        # Pick an interior point nearer the robot for the far bag; fixed normal/yaw.
        offset=self.left_grasp_offset(color,source)
        source=source+offset
        # Shift the released bag 60 mm away from the LCD, within the support footprint.
        resuming=self.objects.get(bag,{}).get('destination')=='scale'
        scale_yaw=float(observations['detections']['scale'].get('yaw_table_rad',0))
        scale_packet=np.array(observations['detections'][bag]['table_xy_m']) if resuming else scale+.06*np.array([-np.sin(scale_yaw),np.cos(scale_yaw)])
        scale_left=scale_packet+offset
        stages=[
            ('pickup_hover',source,.12,None,'left'),('pickup_down',source,source_z,None,'left'),
            ('grasp',source,source_z,'attach','left'),('pickup_up',source,travel,None,'left'),
            ('scale_hover',scale_left,travel,None,'left'),('scale_down',scale_left,scale_z,None,'left'),
            ('scale_release',scale_left,scale_z,'release','left'),('scale_retreat',scale_left,travel,None,'left'),
            ('left_clear',self.left_clear_xy.copy(),travel,None,'left'),
            ('weigh',self.left_clear_xy.copy(),travel,'weigh','left'),
            ('right_transfer_high',scale_packet,scale_transfer_height,None,'right'),
            ('right_scale_hover',scale_packet,travel,None,'right'),
            ('scale_repick_down',scale_packet,scale_z,None,'right'),
            ('scale_regrasp',scale_packet,scale_z,'attach','right'),('scale_repick_up',scale_packet,travel,None,'right'),
            ('bin_raise',scale_packet,scale_transfer_height,None,'right'),
            ('bin_hover',box,bin_travel,None,'right'),('bin_down',box,box_z,None,'right'),
            ('bin_release',box,box_z,'release','right'),('bin_settle',box,box_z,'drop','right'),
            ('bin_retreat',box,bin_travel,None,'right'),
        ]
        return stages[9:] if resuming else stages

    def left_grasp_offset(self,color,source):
        preferred=-.07 if source[1]>.4 else (.03 if source[1]<.2 else 0.)
        return np.array([0.,self.grasp_bias_overrides.get(color,preferred)])

    def cycle(self, colors, execute=False, observations=None, scale_reader=None,source_verifier=None,placement_observer=None,release_environment=None):
        """Search RGB parcel interior grip points, with complete-route preflight."""
        if not observations:raise ValueError('RGB-only observations are required')
        if not colors or any(c not in BAG_COLORS for c in colors):raise ValueError('Choose white, green and/or black parcels')
        tried={color:set() for color in colors};attempts=[]
        for _ in range(12*len(colors)):
            for color in colors:
                source=observations['detections']['bag_'+color]['table_xy_m']
                tried[color].add(float(self.left_grasp_offset(color,source)[1]))
            result=self._cycle(colors,execute,observations,scale_reader,source_verifier,placement_observer,release_environment)
            if result.get('status')!='rejected':
                result['selected_grasp_bias_m']=dict(self.grasp_bias_overrides)
                result['grasp_search_attempts']=attempts
                atomic_json(self.state_dir/'grasp_search.json',{'selected_bias_m':dict(self.grasp_bias_overrides),'attempts':attempts})
                if execute:atomic_json(self.state_dir/'task_result.json',result)
                return result
            failure=result.get('failure',{});color=result.get('bag')
            if failure.get('arm')!='left' or color not in tried:return result
            # 20 mm remains inside the known 200 mm depth at -80 mm; the
            # previous -70 mm lower bound gave away reach near the boundary.
            candidates=[-.055,-.04,-.025,0.,.025,.04,.055,.07,-.07,-.08]
            remaining=[bias for bias in candidates if bias not in tried[color]]
            if not remaining:return result
            source=observations['detections']['bag_'+color]['table_xy_m']
            preferred=float(self.left_grasp_offset(color,source)[1])
            estimated_bias=preferred;selection='nearest untried contact offset'
            ik=failure.get('ik',{})
            if ik.get('collision_free') and ik.get('orientation_error_deg',180)<=1 and ik.get('achieved_xyz_m'):
                desired=np.asarray(failure.get('failed_waypoint_m',failure['requested_xyz_m']))
                axis=np.asarray(self.cal['fit']['affine_world_from_xy'][2],float);axis/=np.linalg.norm(axis)
                forward_error=float((desired-np.asarray(ik['achieved_xyz_m']))@axis)
                if abs(forward_error)>.005:
                    estimated_bias=preferred-forward_error
                    selection='IK position residual projected onto the manually calibrated table depth axis'
            bias=min(remaining,key=lambda v:abs(v-estimated_bias))
            attempts.append({'bag':color,'failed_stage':failure.get('stage'),
                             'old_bias_m':preferred,'new_bias_m':bias,'selection':selection,'published_actions':0})
            self.grasp_bias_overrides[color]=bias
        return result

    def _preflight_route(self, colors, observations):
        original=self.arm._arm14_rad().copy();old_objects=copy.deepcopy(self.objects)
        plans=[]
        sorted_count=sum(o.get('destination')=='bin' for o in self.objects.values())
        try:
            for index,color in enumerate(colors):
                stack_index=sorted_count+index
                for stage,xy,height,event,side in self.stages(color,observations,stack_index):
                    print(f'[preflight] {color}:{stage}',flush=True)
                    xyz=self.calibrated_xyz(xy,height)
                    result=self.path(xyz,side,color+':'+stage,False)
                    if result['status']=='rejected':
                        report={'status':'rejected','bag':color,'failure':result,'published_actions':0,
                                'reason':'Entire pickup/scale/bin route must be feasible before execution'}
                        atomic_json(self.state_dir/'task_preflight.json',report)
                        atomic_json(self.state_dir/'preflight_rejections'/(str(uuid.uuid4())+'.json'),report)
                        return report
                    result.update({'bag':'bag_'+color,'event':event,'table_xy_m':np.asarray(xy).tolist(),'height_m':height})
                    result['vision_observation_file']=observations.get('evidence_file')
                    result['bin_stack_index']=stack_index
                    center=self.calibrated_xyz(observations['detections']['bag_'+color]['table_xy_m'],0)
                    grasp_xy=np.array(observations['detections']['bag_'+color]['table_xy_m'])
                    grasp_xy+=self.left_grasp_offset(color,grasp_xy)
                    result['grasp_offset_world_m']=((center-self.calibrated_xyz(grasp_xy,.012)).tolist()
                        if side=='left' else (-np.asarray(self.cal['fit']['normal_world'])*.012).tolist())
                    plans.append(result)
                    atomic_json(self.state_dir/'preflight_progress.json',
                                {'last_stage':result['name'],'stages_planned':len(plans),'status':'planning'})
                    self.arm.data.qpos[self.arm.qpos_addresses]=result['target_arm14_rad']
                    mujoco.mj_forward(self.arm.model,self.arm.data)
                    if event=='attach':
                        result['initial_neighbour_contact_escape_m']=self.collision_world.set_held(
                            result['bag'],side,self.tcp_offsets[side],result['grasp_offset_world_m'],result['target_arm14_rad'])
                    elif event=='release':
                        self.collision_world.set_held()
                        self.collision_world.relocate_bag(result['bag'],xyz+result['grasp_offset_world_m'])
                    elif event=='drop':
                        base=xyz+result['grasp_offset_world_m']+np.asarray(self.cal['fit']['normal_world'])*(.12+stack_index*.012-.27)
                        self.collision_world.relocate_bag(result['bag'],base)
                        self.collision_world.set_packed_count(stack_index+1)
        finally:
            self.arm.data.qpos[self.arm.qpos_addresses]=original
            mujoco.mj_forward(self.arm.model,self.arm.data)
            self.objects=old_objects
            self.collision_world.set_held()
        return {'status':'planned','stages':plans,'published_actions':0}

    def _cycle(self, colors, execute=False, observations=None, scale_reader=None,source_verifier=None,placement_observer=None,release_environment=None):
        if not self.cal.get('fit'):raise ValueError('Four-point calibration is not fitted; task is locked')
        if not observations:raise ValueError('RGB-only observations are required; simulator object XY is not a localization input')
        self.set_collision_world(observations)
        if any(self.objects.get('bag_'+c,{}).get('destination')=='bin' for c in colors):
            raise ValueError('Requested bag has already been sorted; start an independent session to repeat')
        preflight=self._preflight_route(colors,observations)
        if preflight['status']=='rejected':return preflight
        plans=preflight['stages']
        if not execute:
            report={'status':'planned','stages':plans,'published_actions':0}
            atomic_json(self.state_dir/'task_preflight.json',report)
            atomic_json(self.state_dir/'preflight_progress.json',{'status':'planned','stages_planned':len(plans)})
            return report
        if scale_reader is None:raise ValueError('Task execution requires an RGB scale-display reader; force/mass metadata must not be used as a sensor input')
        weights=[]
        for execution_index,record in enumerate(plans):
            print(f'[execute] {record["name"]}',flush=True)
            name=record['bag'];height=.012  # declared rigid bag proxy height, not scene metadata
            event=record['event']
            if event=='attach':
                self.objects[name]={'attached_arm':record['arm'],'height_m':height,
                                    'offset_world_m':record['grasp_offset_world_m'],
                                    'tcp_local_m':self.tcp_offsets[record['arm']]}
                if record['name'].endswith('scale_regrasp'):self.scale_display_kg=0
            elif event=='release':
                xyz=np.asarray(record['requested_xyz_m'])+record['grasp_offset_world_m']
                self.objects[name]={'attached_arm':None,'height_m':height,'world_xyz_m':xyz.tolist(),
                                    'destination':'bin' if record['name'].endswith('bin_release') else 'scale'}
            elif event=='weigh':
                try:weight=scale_reader(self,name)
                except (ValueError,RuntimeError,OSError) as error:
                    self.save_state(record['name']+':sensor_rejected')
                    result={'status':'stopped','phase':record['name'],'reason':str(error),
                            'published_actions':execution_index,'weights':weights,
                            'recovery':'Parcel remains on the scale; resolve the RGB reading before continuing'}
                    atomic_json(self.state_dir/'task_result.json',result)
                    atomic_json(self.state_dir/'sensor_failures'/(str(uuid.uuid4())+'.json'),result)
                    return result
                record['weighing']=weight;weights.append({'bag':name,**weight})
                self.scale_display_kg=weight['measured_kg']
            elif event=='drop':
                old=self.objects[name]['world_xyz_m']
                xyz=np.asarray(old)+np.asarray(self.cal['fit']['normal_world'])*(.12+record['bin_stack_index']*.012-.27)
                self.objects[name]={'attached_arm':None,'height_m':height,'world_xyz_m':xyz.tolist(),
                                    'drop_from_world_xyz_m':old,'destination':'bin'}
                if release_environment:
                    record['release_environment']=release_environment(self,name,record)
                    old_duration=record['actual_duration_s'];record['actual_duration_s']=max(3.,old_duration)
                    record['trajectory_t_s']=[t*record['actual_duration_s']/old_duration for t in record['trajectory_t_s']]
            record['status']='executed';record['task_objects']=copy.deepcopy(self.objects)
            record['head_yaw_pitch_deg']=[float(np.degrees(self.arm.data.qpos[self.arm.model.joint(j).qposadr[0]])) for j in ['zhead_1_joint','zhead_2_joint']]
            record['scale_display_kg']=self.scale_display_kg
            self.arm.data.qpos[self.arm.qpos_addresses]=record['target_arm14_rad']
            mujoco.mj_forward(self.arm.model,self.arm.data)
            apply_objects(self.arm.model,self.arm.data,self.objects)
            self.save_state(record['name']);self.arm.store.append_history(record)
            atomic_json(self.state_dir/'preflight_progress.json',{'status':'executing','last_stage':record['name'],
                        'stages_planned':len(plans),'stages_executed':execution_index+1})
            if event=='drop':self.objects[name].pop('drop_from_world_xyz_m',None)
            try:
                if record['name'].endswith(':scale_hover') and source_verifier:
                    evidence=source_verifier(self,name)
                    if not evidence.get('verified'):raise ValueError('Source parcel did not visibly clear; grasp/transport is not verified')
                if record['name'].endswith(':left_clear') and placement_observer:
                    updated=placement_observer(self,name)
                    self.set_collision_world(updated)
                    replanned=self._preflight_route([name.removeprefix('bag_')],updated)
                    atomic_json(self.state_dir/'right_replans'/(name+'.json'),replanned)
                    if replanned['status']!='planned':raise ValueError('Post-placement RGB right-arm route rejected: '+json.dumps(replanned.get('failure',{})))
                    plans[execution_index+1:]=replanned['stages']
            except (ValueError,RuntimeError,OSError) as error:
                self.save_state(record['name']+':visual_verification_rejected')
                report={'status':'stopped','phase':record['name'],'reason':str(error),
                        'published_actions':execution_index+1,'weights':weights,
                        'recovery':'Inspect the held parcel or parcel on scale; restore safely before resume'}
                atomic_json(self.state_dir/'task_result.json',report)
                return report
        result={'status':'executed','stages':len(plans),'weights':weights,
                'grasp_mode':'visual kinematic attachment; no finger/contact grasp physics'}
        atomic_json(self.state_dir/'task_result.json',result)
        atomic_json(self.state_dir/'preflight_progress.json',{'status':'executed','stages_executed':len(plans)})
        return result


def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--scene',type=Path,default=SCENE);p.add_argument('--state-dir',type=Path,default=STATE)
    p.add_argument('--palm-yaw',type=float,help='fixed left setup yaw in degrees (new session -10); palm remains down')
    p.add_argument('--right-palm-yaw',type=float,help='fixed right setup yaw in degrees (new session +20); palm remains down')
    s=p.add_subparsers(dest='command',required=True)
    s.add_parser('init');s.add_parser('status');s.add_parser('fit')
    a=s.add_parser('import-calibration');a.add_argument('path',type=Path)
    a=s.add_parser('audit');a.add_argument('--clearance',type=float,default=.30)
    a=s.add_parser('approach');a.add_argument('point',choices=list('ABCD'));a.add_argument('--xy',type=float,nargs=2)
    a.add_argument('--clearance',type=float,default=.30);a.add_argument('--execute',action='store_true')
    a=s.add_parser('record');a.add_argument('point',choices=list('ABCD'));a.add_argument('--confirm',action='store_true')
    a=s.add_parser('jog');a.add_argument('--arm',choices=['left','right'],required=True)
    a.add_argument('--delta',type=float,nargs=3,required=True);a.add_argument('--execute',action='store_true')
    a=s.add_parser('cycle');a.add_argument('--bags',nargs='+',choices=list(BAG_COLORS),default=list(BAG_COLORS))
    a.add_argument('--execute',action='store_true')
    a.add_argument('--rgb',type=Path,required=True,help='RGB image used for all localization (no object ground truth)')
    args=p.parse_args();task=PlanarTask(args.scene,args.state_dir,args.palm_yaw,args.right_palm_yaw)
    if args.command=='init':result=task.initialize()
    elif args.command=='status':result={'calibration':task.cal,'state':task.save_state('manual_status')}
    elif args.command=='audit':result=task.audit(args.clearance)
    elif args.command=='approach':result=task.approach(args.point,args.clearance,args.execute,args.xy)
    elif args.command=='record':result=task.record(args.point,args.confirm)
    elif args.command=='fit':result=task.fit()
    elif args.command=='import-calibration':result=task.import_corners(args.path)
    elif args.command=='cycle':
        import cv2
        try:
            from kuavo_table_vision import FourPointCalibration,TableVision
        except ModuleNotFoundError:
            from tools.kuavo_table_vision import FourPointCalibration,TableVision
        corners={k:v['measured_world_xyz_m'] for k,v in task.cal['points'].items()}
        calibration=FourPointCalibration(corners,task.cal['fit']['calibration_clearance_m'])
        rgb=cv2.cvtColor(cv2.imread(str(args.rgb)),cv2.COLOR_BGR2RGB)
        observation=TableVision(calibration).detect(rgb)
        scale_reader=None
        if args.execute:
            try:
                from kuavo_sim_scale_environment import read_scale_from_environment_rgb
            except ModuleNotFoundError:
                from tools.kuavo_sim_scale_environment import read_scale_from_environment_rgb
            scale_reader=read_scale_from_environment_rgb
        result=task.cycle(args.bags,args.execute,observation,scale_reader)
    else:
        if np.linalg.norm(args.delta)>.05:raise ValueError('Jog displacement must not exceed 5 cm')
        xyz=task.arm._eef(args.arm)[0]+np.array(args.delta)
        result=task.path(xyz,args.arm,'manual_jog',args.execute)
    print(json.dumps(result,ensure_ascii=False,indent=2))
    if isinstance(result,dict) and result.get('status') in ('rejected','stopped'):raise SystemExit(2)


if __name__=='__main__':main()
