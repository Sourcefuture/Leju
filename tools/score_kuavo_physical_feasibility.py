#!/usr/bin/env python3
"""Independent CAD/full-body, load, joint limits and necessary reaction audit.

The score is NOT a finger grasp test or a whole-body controller simulation.
Scene truth/private loads are used only here, never as localization inputs.
"""
from __future__ import annotations
import argparse,json,os
from pathlib import Path
os.environ.setdefault('MUJOCO_GL','egl')
import mujoco,numpy as np
from scipy.optimize import linprog
from kuavo_planar_task import apply_objects,atomic_json,load_json,sample_motion_derivatives
from kuavo_mujoco_live_viewer import arm_addresses
from kuavo_sim_bin_environment import apply_private_bin_poses


def ground_points(m,d):
    floor=m.geom('tabletop_floor').id;points=[]
    feet=[i for i in range(m.ngeom) if m.geom_group[i]==3 and m.body(m.geom_bodyid[i]).name in ['leg_l6_link','leg_r6_link']]
    for ident in feet:
        if abs(mujoco.mj_geomDistance(m,d,ident,floor,.01,None))>.001:continue
        body=int(m.geom_bodyid[ident]);r=d.geom_xmat[ident].reshape(3,3);size=m.geom_size[ident];center=d.geom_xpos[ident]
        if m.geom_type[ident]==mujoco.mjtGeom.mjGEOM_BOX:
            vertices=np.array([center+r@(size*np.array([x,y,z])) for x in [-1,1] for y in [-1,1] for z in [-1,1]])
            points.extend((body,p.copy()) for p in vertices if p[2]<=vertices[:,2].min()+.0005)
        elif m.geom_type[ident]==mujoco.mjtGeom.mjGEOM_CAPSULE:
            # Tangent line for a horizontal capsule; deliberately conservative.
            ends=[center-r[:,2]*size[1],center+r[:,2]*size[1]]
            points.extend((body,p-[0,0,size[0]]) for p in ends if p[2]<=min(v[2] for v in ends)+.0005)
    if not points:raise ValueError('No supported foot primitives')
    return points


def conservative_distances(m,d,pairs,limit=.005):
    pairs=np.asarray(pairs,int).reshape(-1,2)
    delta=d.geom_xpos[pairs[:,0]]-d.geom_xpos[pairs[:,1]]
    radii=m.geom_rbound[pairs[:,0]]+m.geom_rbound[pairs[:,1]]+limit
    result=[]
    for index in np.flatnonzero(np.einsum('ij,ij->i',delta,delta)<=radii*radii):
        i,j=map(int,pairs[index]);v=mujoco.mj_geomDistance(m,d,i,j,limit,None)
        if v<-.002:result.append((i,j,float(-v*1000)))
    return result


def score(scene,history,friction=.4,sample_stride=1):
    m=mujoco.MjModel.from_xml_path(str(Path(scene).resolve()));d=mujoco.MjData(m)
    mujoco.mj_resetDataKeyframe(m,d,0);mujoco.mj_forward(m,d)
    addresses=arm_addresses(m)
    arm_dofs=np.array([m.joint(f'zarm_{s}{i}_joint').dofadr[0] for s in ['l','r'] for i in range(1,8)])
    root_dofs=np.arange(6);points=ground_points(m,d)
    contacts=[]
    for body,point in points:
        jac=np.zeros((3,m.nv));rot=np.zeros((3,m.nv));mujoco.mj_jac(m,d,jac,rot,point,body)
        contacts.append(jac.T)
    support=np.concatenate(contacts,axis=1)
    loads=load_json(Path(scene).parent/'environment_loads.json',{})
    robot=[i for i in range(m.ngeom) if m.geom_group[i]==3 and int(m.geom_contype[i])&1]
    if not robot:
        robot=[i for i in range(m.ngeom) if m.geom_group[i]==3 and not (m.geom(i).name or '').startswith(('observed_','filler_'))]
    objects=[i for i in range(m.ngeom) if int(m.geom_contype[i])&8 and m.geom(i).name!='tabletop_floor']
    motor_dofs=[];capacities=[];motor_names=[]
    for ident in range(m.nu):
        joint=int(m.actuator_trnid[ident,0])
        if joint<0 or not m.actuator_ctrllimited[ident]:continue
        motor_dofs.append(int(m.jnt_dofadr[joint]));motor_names.append(m.joint(joint).name)
        capacities.append(float(np.max(abs(m.actuator_ctrlrange[ident]))*abs(m.actuator_gear[ident,0])))
    motor_dofs=np.array(motor_dofs,int);capacities=np.array(capacities)
    # Inscribed friction pyramid: |Fx|+|Fy| <= mu Fz, Fz >= 0.
    cone=[]
    for index in range(len(points)):
        for x,y in [(-1,-1),(-1,1),(1,-1),(1,1)]:
            row=np.zeros(3*len(points));row[index*3:index*3+3]=[x,y,-friction];cone.append(row)
    cone=np.array(cone)
    limits=[];rigid=[];cloth=[];reaction_failures=[];load_failures=[]
    max_arm_ratio=0.;max_arm_tau=0.;max_speed=0.;max_acc=0.;max_jerk=0.;samples=0;worst=None
    records=[json.loads(line) for line in Path(history).read_text().splitlines()]
    records=[r for r in records if r.get('kind')=='planar_task' and r.get('status')=='executed']
    for record in records:
        states=record.get('task_objects',{});held={m.body(n).id for n,s in states.items() if s.get('attached_arm')}
        pairs=[(i,j) for i in robot for j in objects if not (m.geom_bodyid[j] in held and m.body(m.geom_bodyid[i]).name.endswith('7_link'))]
        pairs += [(i,j) for i in objects if m.geom_bodyid[i] in held for j in objects if m.geom_bodyid[j] not in held]
        path=np.asarray(record['trajectory_arm14_rad']);times=np.asarray(record.get('trajectory_t_s',np.linspace(0,record['actual_duration_s'],len(path))))
        changes=abs(np.degrees(np.diff(path,axis=0)));intervals=np.diff(times)
        smooth=record.get('trajectory_profile')=='quintic_stop_at_waypoints'
        max_speed=max(max_speed,float(np.max(changes/intervals[:,None]))*(1.875 if smooth else 1))
        if smooth:
            max_acc=max(max_acc,float(np.max(5.773502692*changes/intervals[:,None]**2)))
            max_jerk=max(max_jerk,float(np.max(60*changes/intervals[:,None]**3)))
        for segment in range(0,len(path)-1,sample_stride):
            for fraction in [0.,.2113248654,.5,.7886751346,1.]:
                t=times[segment]+fraction*(times[segment+1]-times[segment]);q,vel,acc=sample_motion_derivatives(record,t/times[-1])
                d.qpos[addresses]=q;d.qvel[:]=0;d.qvel[arm_dofs]=vel;mujoco.mj_forward(m,d)
                if record.get('head_yaw_pitch_deg'):
                    for name,v in zip(['zhead_1_joint','zhead_2_joint'],record['head_yaw_pitch_deg']):d.qpos[m.joint(name).qposadr[0]]=np.radians(v)
                    mujoco.mj_forward(m,d)
                apply_objects(m,d,states)
                apply_private_bin_poses(m,d,states,Path(history).parent,fraction,
                    record.get('bag') if record.get('event')=='drop' else None,record.get('action_id'))
                ids=np.array([m.joint(f'zarm_{s}{i}_joint').id for s in ['l','r'] for i in range(1,8)])
                if np.any(q<m.jnt_range[ids,0]-1e-7) or np.any(q>m.jnt_range[ids,1]+1e-7):limits.append(record['name'])
                for first,second,penetration in conservative_distances(m,d,pairs):
                    entry={'stage':record['name'],'robot_body':m.body(m.geom_bodyid[first]).name,'obstacle':m.geom(second).name,'penetration_mm':penetration}
                    (cloth if (m.geom(second).name or '').startswith('cloth_') else rigid).append(entry)
                d.qacc[:]=0;d.qacc[arm_dofs]=acc if smooth else 0
                required=np.zeros(m.nv);mujoco.mj_rne(m,d,1,required)
                required+=m.dof_armature*d.qacc-d.qfrc_passive
                required+=m.dof_frictionloss*np.where(abs(d.qvel)>1e-6,np.sign(d.qvel),0)
                for name,obj in states.items():
                    if not obj.get('attached_arm'):continue
                    mass=loads.get(name)
                    if mass is None:load_failures.append({'stage':record['name'],'reason':'missing private scoring load '+name});continue
                    body=m.body('zarm_'+('l' if obj['attached_arm']=='left' else 'r')+'7_link').id
                    p=d.body(name).xpos;ja=np.zeros((3,m.nv));jr=np.zeros((3,m.nv));jdot=np.zeros((3,m.nv));rdot=np.zeros((3,m.nv))
                    mujoco.mj_jac(m,d,ja,jr,p,body);mujoco.mj_jacDot(m,d,jdot,rdot,p,body)
                    acceleration=ja@d.qacc+jdot@d.qvel if smooth else np.zeros(3)
                    required+=ja.T@(float(mass)*(acceleration+[0,0,9.81]))
                arm_rows=[i for i,dof in enumerate(motor_dofs) if dof in arm_dofs]
                ratios=abs(required[motor_dofs[arm_rows]])/capacities[arm_rows]
                if max(ratios)>max_arm_ratio:
                    max_arm_ratio=float(max(ratios));idx=arm_rows[int(np.argmax(ratios))]
                    worst={'stage':record['name'],'joint':motor_names[idx],'required_torque_nm':float(required[motor_dofs[idx]]),'model_limit_nm':float(capacities[idx])}
                max_arm_tau=max(max_arm_tau,float(np.max(abs(required[arm_dofs]))))
                # Feasible ground forces must satisfy floating-base dynamics
                # AND model motor limits, rather than COM projection alone.
                a_ub=np.r_[cone,support[motor_dofs],-support[motor_dofs]]
                b_ub=np.r_[np.zeros(len(cone)),required[motor_dofs]+capacities,-required[motor_dofs]+capacities]
                bounds=[(None,None) if i%3!=2 else (0,None) for i in range(support.shape[1])]
                result=linprog(np.zeros(support.shape[1]),A_ub=a_ub,b_ub=b_ub,A_eq=support[root_dofs],b_eq=required[root_dofs],bounds=bounds,method='highs')
                if not result.success:
                    reaction_failures.append({'stage':record['name'],'segment':segment,'fraction':fraction,'reason':result.message})
                samples+=1
        if len(rigid)>200 or len(reaction_failures)>200:break
    return {'passed_necessary_checks':bool(records) and not (limits or rigid or reaction_failures or load_failures) and max_speed<=25.00001 and (not max_acc or max_acc<=90.00001) and (not max_jerk or max_jerk<=500.00001),
        'actions':len(records),'samples':samples,'sample_stride':sample_stride,'friction_assumption':friction,
        'rigid_collision_count':len(rigid),'rigid_collisions':rigid[:30],
        'cloth_intersection_count':len(cloth),'cloth_intersections':cloth[:20],
        'reaction_failure_count':len(reaction_failures),'reaction_failures':reaction_failures[:20],
        'joint_limit_failures':limits[:20],'load_failures':load_failures[:20],
        'max_arm_torque_ratio':max_arm_ratio,'max_arm_torque_nm':max_arm_tau,'worst_arm_load':worst,
        'max_joint_speed_deg_s':max_speed,'max_joint_acceleration_deg_s2':max_acc,'max_joint_jerk_deg_s3':max_jerk,
        'trajectory_dynamics':'quintic arm velocity/acceleration included with point-mass payload; legacy linear records checked quasi-statically and cannot prove acceleration limits',
        'scope':'necessary discrete CAD whole-body/environment and model motor/reaction feasibility; no executed WBC, articulated fingers, cloth or soft-package dynamics, actuator tracking, real friction or real payload capacity validation'}


def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--scene',type=Path,required=True);p.add_argument('--history',type=Path,required=True);p.add_argument('--output',type=Path,required=True)
    p.add_argument('--friction',type=float,default=.4);p.add_argument('--sample-stride',type=int,default=1)
    a=p.parse_args()
    if not .05<=a.friction<=1 or a.sample_stride<1:p.error('friction .05..1 and positive sample stride required')
    result=score(a.scene,a.history,a.friction,a.sample_stride);atomic_json(a.output,result)
    print(json.dumps({k:v for k,v in result.items() if not isinstance(v,list)},indent=2),flush=True)
    if not result['passed_necessary_checks']:raise SystemExit(2)


if __name__=='__main__':main()
