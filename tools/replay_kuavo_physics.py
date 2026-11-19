#!/usr/bin/env python3
"""Actual mj_step replay: free rigid parcels, torque-actuated floating robot.

Experimental foot-contact inverse-dynamics tracking is not the official WBC.
No fingers, weld or kinematic parcel attachment are supplied in this test.
"""
from __future__ import annotations
import argparse,copy,json,os,xml.etree.ElementTree as ET
from pathlib import Path
os.environ.setdefault('MUJOCO_GL','egl')
import cv2,mujoco,numpy as np
from scipy.optimize import linprog,minimize,LinearConstraint,Bounds
from kuavo_planar_task import atomic_json,load_json,sample_motion_derivatives
from kuavo_mujoco_live_viewer import arm_addresses
from score_kuavo_physical_feasibility import ground_points
from kuavo_table_vision import TableVision,FourPointCalibration
from kuavo_visual_verification import SourceClearanceVerifier


def make_plant(scene):
    scene=Path(scene).resolve();original=mujoco.MjModel.from_xml_path(str(scene));od=mujoco.MjData(original)
    mujoco.mj_resetDataKeyframe(original,od,0);mujoco.mj_forward(original,od)
    tree=ET.parse(scene);xml=tree.getroot();world=xml.find('worldbody');table=world.find("body[@name='task_table']")
    keys=xml.find('keyframe')
    if keys is not None:xml.remove(keys)
    loads=load_json(scene.parent/'environment_loads.json',{})
    for name in ['bag_white','bag_green','bag_black']:
        body=table.find(f"body[@name='{name}']");table.remove(body)
        body.set('pos',' '.join(map(str,od.body(name).xpos)));body.set('quat',' '.join(map(str,od.body(name).xquat)))
        ET.SubElement(body,'freejoint',name=name+'_physics_free')
        for geom in body.findall('geom'):
            if 'layer' in geom.get('name',''):geom.set('mass',str(float(loads.get(name,.3))/3))
            else:geom.set('density','0')
        world.append(body)
    # Task objects use category 8, robot contact primitives category 1. Enable
    # physical robot/object/floor interactions that kinematic scenes bypass.
    for geom in xml.findall(".//geom[@group='3']"):
        geom.set('conaffinity','9')
    m=mujoco.MjModel.from_xml_string(ET.tostring(xml,encoding='unicode'));d=mujoco.MjData(m)
    d.qpos[:original.nq]=od.qpos;mujoco.mj_forward(m,d)
    return m,d,original.nq


class ExperimentalFootTracker:
    def __init__(self,m,d):
        self.m=m;self.d=d;self.q0=d.qpos.copy();self.root=d.qpos[:7].copy();self.previous_force=None
        self.joints=[int(m.actuator_trnid[i,0]) for i in range(m.nu)]
        self.dofs=np.array([m.jnt_dofadr[j] for j in self.joints],int);self.qadr=np.array([m.jnt_qposadr[j] for j in self.joints],int)
        self.cap=np.max(abs(m.actuator_ctrlrange),axis=1)*abs(m.actuator_gear[:,0]);self.failures=0
        self.feet=[]
        for s in ['l','r']:
            body=m.body('leg_'+s+'6_link').id;self.feet.append((body,d.xpos[body].copy(),d.xquat[body].copy()))
        self.points=[]
        for body,point in ground_points(m,d):
            self.points.append((body,d.xmat[body].reshape(3,3).T@(point-d.xpos[body])))
        self.cone=[]
        for i in range(len(self.points)):
            for x,y in [(-1,-1),(-1,1),(1,-1),(1,1)]:
                row=np.zeros(len(self.points)*3);row[i*3:i*3+3]=[x,y,-.4];self.cone.append(row)
        self.cone=np.array(self.cone)

    def control(self,target,velocity,acceleration):
        m,d=self.m,self.d;mujoco.mj_forward(m,d)
        wanted=np.zeros(m.nv)
        wanted[self.dofs]=acceleration+400*(target-d.qpos[self.qadr])+40*(velocity-d.qvel[self.dofs])
        wanted[:3]=80*(self.root[:3]-d.qpos[:3])-18*d.qvel[:3]
        error=np.zeros(3);mujoco.mju_subQuat(error,self.root[3:7],d.qpos[3:7]);wanted[3:6]=100*error-20*d.qvel[3:6]
        rows=[];rhs=[]
        for body,pos,quat in self.feet:
            ja=np.zeros((3,m.nv));jr=np.zeros((3,m.nv));jd=np.zeros((3,m.nv));rd=np.zeros((3,m.nv))
            mujoco.mj_jac(m,d,ja,jr,d.xpos[body],body);mujoco.mj_jacDot(m,d,jd,rd,d.xpos[body],body)
            dr=np.zeros(3);mujoco.mju_subQuat(dr,quat,d.xquat[body])
            rows.extend([ja,jr]);rhs.extend([-jd@d.qvel+80*(pos-d.xpos[body])-18*(ja@d.qvel),-rd@d.qvel+100*dr-20*(jr@d.qvel)])
        j=np.concatenate(rows);b=np.concatenate(rhs)
        # Feet and base/posture objectives are reconciled in acceleration space.
        wanted-=j.T@np.linalg.solve(j@j.T+1e-6*np.eye(len(j)),j@wanted-b)
        d.qacc[:]=wanted;required=np.zeros(m.nv);mujoco.mj_rne(m,d,1,required)
        # RNE omits passive damping/springs and dry joint friction. Without
        # compensation low-inertia wrists can stick many degrees behind the
        # target even when floating-base balance is excellent.
        # mj_rne excludes rotor/armature inertia; verified against mj_fullM.
        required+=m.dof_armature*wanted
        required-=d.qfrc_passive
        drive=velocity+10*(target-d.qpos[self.qadr])
        required[self.dofs]+=m.dof_frictionloss[self.dofs]*np.tanh(drive/.02)
        contact=[]
        for body,local in self.points:
            point=d.xpos[body]+d.xmat[body].reshape(3,3)@local
            ja=np.zeros((3,m.nv));jr=np.zeros((3,m.nv));mujoco.mj_jac(m,d,ja,jr,point,body);contact.append(ja.T)
        support=np.concatenate(contact,axis=1)
        result=linprog(np.zeros(support.shape[1]),A_eq=support[:6],b_eq=required[:6],
            A_ub=np.r_[self.cone,support[self.dofs],-support[self.dofs]],
            b_ub=np.r_[np.zeros(len(self.cone)),required[self.dofs]+self.cap,-required[self.dofs]+self.cap],
            bounds=[(None,None) if i%3!=2 else (0,None) for i in range(support.shape[1])],method='highs')
        if result.success:
            # A zero-cost LP often concentrates forces at friction/torque
            # limits. Choose a smooth, minimum-norm interior distribution.
            matrix=np.r_[support[:6],self.cone,support[self.dofs]]
            lo=np.r_[required[:6],np.full(len(self.cone),-np.inf),required[self.dofs]-self.cap]
            hi=np.r_[required[:6],np.zeros(len(self.cone)),required[self.dofs]+self.cap]
            scale=100.;previous=self.previous_force if self.previous_force is not None else result.x
            solution=minimize(lambda f:float((f@f+.05*((f-previous)@(f-previous)))/(scale*scale)),result.x,
                jac=lambda f:2*(f+.05*(f-previous))/(scale*scale),method='SLSQP',
                constraints=[LinearConstraint(matrix,lo,hi)],
                bounds=Bounds([0 if i%3==2 else -np.inf for i in range(support.shape[1])],np.full(support.shape[1],np.inf)),
                options={'maxiter':40,'ftol':1e-8})
            self.previous_force=solution.x if solution.success else result.x
        else:self.failures+=1
        force=self.previous_force if self.previous_force is not None else np.zeros(support.shape[1])
        torque=(required-support@force)[self.dofs]
        d.ctrl[:]=np.clip(torque/m.actuator_gear[:,0],m.actuator_ctrlrange[:,0],m.actuator_ctrlrange[:,1])
        return float(np.max(abs(torque)/self.cap))


def run(args):
    records=[json.loads(line) for line in args.history.read_text().splitlines()]
    records=[r for r in records if r.get('kind')=='planar_task' and r.get('status')=='executed' and r.get('bag')=='bag_'+args.bag]
    if not records:raise ValueError('No matching recorded planar actions')
    selected=[]
    for record in records:
        selected.append(record)
        if record['name'].endswith(':'+args.until):break
    m,d,robot_nq=make_plant(args.scene);addresses=arm_addresses(m);d.qpos[addresses]=selected[0]['trajectory_arm14_rad'][0]
    for name,v in zip(['zhead_1_joint','zhead_2_joint'],selected[0].get('head_yaw_pitch_deg',[])):d.qpos[m.joint(name).qposadr[0]]=np.radians(v)
    mujoco.mj_forward(m,d);tracker=ExperimentalFootTracker(m,d)
    motor_map={m.joint(j).name:i for i,j in enumerate(tracker.joints)}
    arm_motor=np.array([motor_map[f'zarm_{s}{i}_joint'] for s in ['l','r'] for i in range(1,8)])
    targets=tracker.q0[tracker.qadr].copy();bagid=m.body('bag_'+args.bag).id;initial_bag=d.xpos[bagid].copy()
    initial_robot=d.qpos[:7].copy();max_root_error=0.;max_arm_error=0.;max_ratio=0.;max_bag_lift=0.;samples=0;fell=False
    reference_data=mujoco.MjData(m);reference_data.qpos[:]=d.qpos
    tcp_offsets=load_json(args.history.parent/'latest_state.json',{}).get('contact_tcp_offsets_local_m',{'left':[0,-.045,-.193],'right':[0,.045,-.193]})
    max_tcp_error=0.;max_palm_error=0.
    args.output.parent.mkdir(parents=True,exist_ok=True)
    writer=cv2.VideoWriter(str(args.output),cv2.VideoWriter_fourcc(*'mp4v'),args.fps,(1280,720))
    if not writer.isOpened():raise RuntimeError('Video writer failed')
    elapsed=0.;next_frame=0.;next_control=0.;last_name='settling';rows=[];last_head=[]
    cam=m.camera('sim_head').id
    try:
        with mujoco.Renderer(m,height=480,width=848) as renderer:
            for record in selected:
                duration=float(record['actual_duration_s']);local=0.;last_name=record['name']
                print('physics',last_name,flush=True)
                while local<duration:
                    q,v,a=sample_motion_derivatives(record,local/duration);targets[arm_motor]=q
                    vv=np.zeros(m.nu);aa=np.zeros(m.nu);vv[arm_motor]=v;aa[arm_motor]=a
                    if elapsed>=next_control:
                        max_ratio=max(max_ratio,tracker.control(targets,vv,aa));next_control=elapsed+.01
                    mujoco.mj_step(m,d);local+=m.opt.timestep;elapsed+=m.opt.timestep;samples+=1
                    root_error=float(np.linalg.norm(d.qpos[:3]-initial_robot[:3]));arm_error=float(np.max(abs(np.degrees(d.qpos[addresses]-q))))
                    max_root_error=max(max_root_error,root_error);max_arm_error=max(max_arm_error,arm_error)
                    max_bag_lift=max(max_bag_lift,float(d.xpos[bagid,2]-initial_bag[2]))
                    error=np.zeros(3);mujoco.mju_subQuat(error,initial_robot[3:7],d.qpos[3:7])
                    fell=d.qpos[2]<initial_robot[2]-.15 or np.linalg.norm(error)>np.radians(20) or not np.isfinite(d.qpos).all()
                    if elapsed>=next_frame:
                        reference_data.qpos[addresses]=q;mujoco.mj_kinematics(m,reference_data)
                        for side,s,normal in [('left','l',[0,-1,0]),('right','r',[0,1,0])]:
                            body=m.body('zarm_'+s+'7_link').id;offset=np.asarray(tcp_offsets[side])
                            actual=d.xpos[body]+d.xmat[body].reshape(3,3)@offset
                            nominal=reference_data.xpos[body]+reference_data.xmat[body].reshape(3,3)@offset
                            max_tcp_error=max(max_tcp_error,float(np.linalg.norm(actual-nominal)*1000))
                            direction=d.xmat[body].reshape(3,3)@normal
                            max_palm_error=max(max_palm_error,float(np.degrees(np.arccos(np.clip(direction@[0,0,-1],-1,1)))))
                        renderer.update_scene(d,camera='weighing_overview');world=cv2.cvtColor(renderer.render(),cv2.COLOR_RGB2BGR)
                        renderer.update_scene(d,camera='sim_head');head=cv2.cvtColor(renderer.render(),cv2.COLOR_RGB2BGR);last_head.append(cv2.cvtColor(head,cv2.COLOR_BGR2RGB));last_head=last_head[-3:]
                        canvas=np.full((720,1280,3),24,np.uint8);canvas[90:543,:800]=cv2.resize(world,(800,453));canvas[90:362,800:]=cv2.resize(head,(480,272))
                        for y,text in [(30,'ACTUAL mj_step | experimental foot torque tracking | NO fingers/weld/parcel attachment'),(65,f'{last_name} | t {elapsed:.1f}s | floating base error {root_error*1000:.1f} mm'),(580,f'Parcel lift {max_bag_lift*1000:.1f} mm | arm tracking max {arm_error:.2f} deg'),(625,'This is a physical counterexample, not a successful weighing/sorting replay.'),(670,'Robot is not reset to commanded qpos each step. Parcel motion comes from gravity/contact only.')]:
                            cv2.putText(canvas,text,(14,y),cv2.FONT_HERSHEY_SIMPLEX,.56,(210,230,240),1)
                        writer.write(canvas);next_frame=elapsed+1/args.fps
                    if fell:break
                rows.append({'stage':last_name,'time_s':elapsed,'root_error_mm':root_error*1000,'arm_error_deg':arm_error,'bag_world_xyz_m':d.xpos[bagid].tolist()})
                if fell:break
            renderer.update_scene(d,camera='sim_head');rgb=renderer.render().copy();cv2.imwrite(str(args.output.with_name('physics_final_head.png')),cv2.cvtColor(rgb,cv2.COLOR_RGB2BGR))
    finally:writer.release()
    source_report=None
    try:
        observed=load_json(Path(records[0].get('vision_observation_file') or args.history.parent/'observations'/args.bag/'observation.json'))
        corners=load_json(args.corners)['corners_robot'];vision=TableVision(FourPointCalibration(corners))
        reference_file=observed.get('reference_rgb_file') or args.history.parent/'observations'/args.bag/'00.png'
        reference=cv2.cvtColor(cv2.imread(str(reference_file)),cv2.COLOR_BGR2RGB)
        verifier=SourceClearanceVerifier(vision,reference,observed['detections']['bag_'+args.bag],observed['detections'])
        source_report=verifier.verify_sequence(last_head)
    except (ValueError,OSError) as error:source_report={'verified':False,'reason':str(error)}
    result={'mode':'actual mj_step with floating robot, torque motors and free rigid parcels; no weld or finger/grasp action',
        'physics_steps':samples,'elapsed_simulated_s':elapsed,'records_replayed':len(rows),'fell':bool(fell),
        'max_floating_base_position_error_mm':max_root_error*1000,'max_arm_tracking_error_deg':max_arm_error,
        'max_TCP_tracking_error_mm':max_tcp_error,'max_palm_normal_error_deg':max_palm_error,
        'max_unclipped_torque_ratio':max_ratio,'inverse_dynamics_lp_failures':tracker.failures,
        'max_actual_parcel_lift_mm':max_bag_lift*1000,'initial_parcel_world_xyz_m':initial_bag.tolist(),
        'final_parcel_world_xyz_m':d.xpos[bagid].tolist(),'source_RGB_verification':source_report,'stages':rows,
        'grasp_succeeded':False,'scope':'experimental physical counterexample; not official WBC, real robot tracking or soft/finger grasp validation'}
    atomic_json(args.output.with_suffix('.json'),result);print(json.dumps({k:v for k,v in result.items() if k!='stages'},indent=2),flush=True)


def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--scene',type=Path,required=True);p.add_argument('--history',type=Path,required=True);p.add_argument('--corners',type=Path,required=True)
    p.add_argument('--output',type=Path,required=True);p.add_argument('--bag',choices=['white','green','black'],default='white')
    p.add_argument('--until',default='scale_hover');p.add_argument('--fps',type=int,default=6)
    args=p.parse_args()
    if args.fps<1:p.error('positive fps required')
    run(args)


if __name__=='__main__':main()
