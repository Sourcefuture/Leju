#!/usr/bin/env python3
"""Kuavo-5 simulation: four supplied corners + repeated head RGB observations."""
from __future__ import annotations
import argparse,json,os,uuid,hashlib
from pathlib import Path
os.environ.setdefault('MUJOCO_GL','egl')
import cv2,mujoco,numpy as np
from kuavo_planar_task import PlanarTask,atomic_json,load_json
from kuavo_sim_camera import KuavoSimCamera
from kuavo_table_vision import FourPointCalibration,TableVision,VisionError
from kuavo_sim_scale_environment import read_scale_from_environment_rgb
from kuavo_rgb_disturbance import perturb_rgb,apply_conditions
from kuavo_visual_verification import SourceClearanceVerifier
from kuavo_sim_bin_environment import settle as settle_bin_environment


class SimHeadRGBSource:
    """Environment adapter exposes RGB only; no depth, intrinsics or object poses."""
    def __init__(self,scene,state_dir,noise_level=0,roll_deg=0,seed=0,conditions=None):
        self.scene=scene;self.state_dir=state_dir
        self.noise_level=noise_level;self.roll_deg=roll_deg;self.rng=np.random.default_rng(seed)
        self.conditions=conditions or {};self.last_view=None;self.manifest=[]

    def frames(self,output,max_frames=30):
        reader=KuavoSimCamera(self.scene,self.state_dir)
        output.mkdir(parents=True,exist_ok=True)
        self.manifest=[]
        try:
            camera=reader.model.camera('sim_head').id
            angle=np.radians(self.roll_deg)/2
            original=reader.model.cam_quat[camera].copy()
            mujoco.mju_mulQuat(reader.model.cam_quat[camera],original,np.array([np.cos(angle),0,0,np.sin(angle)]))
            # Preset active viewing angles, independent of simulator target positions.
            views=[(0,10),(-5,10),(5,10),(-10,10),(10,10),(0,5),(-10,5),(10,5),
                   (0,15),(-15,15),(15,15),(0,8),(-8,8),(8,8),(0,12),
                   (0,0),(-10,0),(10,0),(0,-5),(-15,-5),(15,-5),
                   (0,20),(-10,20),(10,20),(0,25),(-15,25),(15,25),
                   (0,-10),(-20,5),(20,5)]
            for index,(yaw,pitch) in enumerate(views[:max_frames]):
                yaw+=float(self.conditions.get('head_yaw_bias_deg',0));pitch+=float(self.conditions.get('head_pitch_bias_deg',0))
                angles=[(reader.model.joint(name).id,value) for name,value in [('zhead_1_joint',yaw),('zhead_2_joint',pitch)]]
                if any(not reader.model.jnt_range[j,0]<=np.radians(v)<=reader.model.jnt_range[j,1] for j,v in angles):continue
                reader.data.qpos[reader.model.joint('zhead_1_joint').qposadr[0]]=np.radians(yaw)
                reader.data.qpos[reader.model.joint('zhead_2_joint').qposadr[0]]=np.radians(pitch)
                mujoco.mj_forward(reader.model,reader.data)
                rgb=reader.capture_rgb('head');rgb=perturb_rgb(rgb,self.rng,self.noise_level)
                rgb=apply_conditions(rgb,self.rng,self.conditions)
                cv2.imwrite(str(output/f'{index:02d}.png'),cv2.cvtColor(rgb,cv2.COLOR_RGB2BGR))
                self.last_view=[yaw,pitch];self.manifest.append({'file':f'{index:02d}.png','head_yaw_pitch_deg':self.last_view})
                yield rgb
        finally:
            atomic_json(output/'acquisition.json',{'frames':self.manifest,'noise_level':self.noise_level,
                        'optical_roll_deg':self.roll_deg,'conditions':self.conditions,
                        'purpose':'acquisition/replay evidence; these controls are not camera/object poses fed to localization'})
            reader.close()


def reacquire(source,vision,required,output,on_scale=None):
    frames=[];reason='No RGB received'
    for rgb in source.frames(output):
        frames.append(rgb)
        if len(frames)<3:continue
        try:
            observation=vision.detect_sequence(frames,minimum=3,agreement_m=.015,required=required,on_scale=on_scale)
            observation['head_last_command_deg']=source.last_view
            reference_index=observation['consensus']['accepted_frame_indices'][0]
            observation['reference_rgb_file']=str((output/source.manifest[reference_index]['file']).resolve())
            observation['evidence_file']=str((output/'observation.json').resolve())
            atomic_json(output/'observation.json',observation)
            return observation
        except VisionError as error:reason=str(error)
    raise VisionError('Multi-view reacquisition failed; motion locked: '+reason)


def record_vision_failure(state_dir,results,error,bag=None):
    completed=[r['bag'] for r in results if r['result'].get('status')=='executed']
    report={'status':'stopped' if completed else 'rejected','phase':'vision_reacquisition',
            'bag':bag,'reason':str(error),'results':results,'completed_bags':completed,
            'published_actions_for_this_parcel':0}
    atomic_json(state_dir/'closed_loop_result.json',report)
    print(json.dumps({k:v for k,v in report.items() if k!='results'},ensure_ascii=False),flush=True)
    raise SystemExit(2)


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--scene',type=Path,required=True)
    parser.add_argument('--corners',type=Path,required=True)
    parser.add_argument('--state-dir',type=Path,required=True)
    parser.add_argument('--bags',nargs='+',choices=['white','green','black'],default=['white','green','black'])
    parser.add_argument('--execute',action='store_true',help='Execute locally in kinematic simulation only')
    parser.add_argument('--noise-level',type=int,choices=[0,1,2],default=0)
    parser.add_argument('--camera-roll-deg',type=float,default=0)
    parser.add_argument('--seed',type=int,default=0)
    parser.add_argument('--left-palm-yaw',type=float)
    parser.add_argument('--right-palm-yaw',type=float)
    parser.add_argument('--resume',action='store_true',help='Continue recorded state after a scale read rejection; skip completed parcels')
    parser.add_argument('--marker-layout',type=Path,help='Known printed table marker centres in metres, not scene object positions')
    parser.add_argument('--view-conditions',type=Path,help='Declared camera/image disturbances, no target geometry')
    parser.add_argument('--tcp-offsets',type=Path,help='Measured robot tool configuration: left/right wrist-local XYZ metres')
    args=parser.parse_args()
    print(json.dumps({'phase':'starting','render_backend':os.environ.get('MUJOCO_GL'),
                      'software_rendering':os.environ.get('LIBGL_ALWAYS_SOFTWARE','false'),
                      'scene':str(args.scene),'state_dir':str(args.state_dir)},ensure_ascii=False),flush=True)
    try:
        run(args)
    except (Exception,KeyboardInterrupt) as error:
        report={'status':'interrupted' if isinstance(error,KeyboardInterrupt) else 'error',
                'error_type':type(error).__name__,'reason':str(error),
                'preflight_progress':load_json(args.state_dir/'preflight_progress.json',{})}
        atomic_json(args.state_dir/'closed_loop_error.json',report)
        print(json.dumps(report,ensure_ascii=False),flush=True)
        raise


def run(args):
    print('[vision] acquiring initial RGB observations',flush=True)
    attempt_id=uuid.uuid4().hex[:12]
    observation_root=args.state_dir/'observations'/attempt_id
    code_root=Path(__file__).resolve().parent
    manifest={'attempt_id':attempt_id,'args':{k:str(v) if isinstance(v,Path) else v for k,v in vars(args).items()},
        'code_sha256':{p.name:hashlib.sha256(p.read_bytes()).hexdigest() for p in code_root.glob('kuavo_*.py')},
        'observation_root':str(observation_root.resolve()),'status':'started'}
    atomic_json(args.state_dir/'runs'/(attempt_id+'.json'),manifest)
    evidence_files=load_json(args.state_dir/'latest_observation_files.json',{})
    offsets=load_json(args.tcp_offsets) if args.tcp_offsets else None
    if args.tcp_offsets and offsets is None:raise ValueError('TCP offsets file does not exist')
    task=PlanarTask(args.scene,args.state_dir,args.left_palm_yaw,args.right_palm_yaw,offsets);task.import_corners(args.corners)
    calibration=FourPointCalibration({k:v['measured_world_xyz_m'] for k,v in task.cal['points'].items()},
                                     task.cal['fit']['calibration_clearance_m'])
    layout=load_json(args.marker_layout)['table_centers_m'] if args.marker_layout else None
    conditions=load_json(args.view_conditions,{}) if args.view_conditions else {}
    vision=TableVision(calibration,table_markers=layout);source=SimHeadRGBSource(args.scene,args.state_dir,args.noise_level,args.camera_roll_deg,args.seed,conditions)
    previous=load_json(args.state_dir/'closed_loop_result.json',{}) if args.resume else {}
    latest={r['bag']:r for r in previous.get('results',[])}
    results=list(latest.values());attempts=previous.get('attempts',[])
    if args.resume:
        if any(o.get('attached_arm') for o in task.objects.values()):raise ValueError('A held parcel requires manual recovery; automatic resume supports only parcels on scale or in bin')
        args.bags=[c for c in args.bags if task.objects.get('bag_'+c,{}).get('destination')!='bin']
        if not args.bags:
            print(json.dumps({'status':'already_completed','published_actions':0}));return
    on_scale=['bag_'+c for c in args.bags if task.objects.get('bag_'+c,{}).get('destination')=='scale']
    required=['bag_'+c for c in args.bags]+['scale','material_box']
    try:first=reacquire(source,vision,required,observation_root/('resume_initial' if args.resume else 'initial'),on_scale)
    except VisionError as error:record_vision_failure(args.state_dir,results,error)
    task.set_head_command(first['head_last_command_deg'])
    evidence_files['initial']=first['evidence_file'];atomic_json(args.state_dir/'latest_observation_files.json',evidence_files)
    # A simulation reset; never represent this as a validated real robot transition.
    if not args.resume:task.initialize(first)
    if not args.execute:
        report=task.cycle(args.bags,False,first)
        atomic_json(args.state_dir/'closed_loop_result.json',report)
        print(json.dumps({'status':report['status'],'detail':str(args.state_dir/'task_preflight.json')},ensure_ascii=False))
        return
    for index,color in enumerate(args.bags):
        required=['bag_'+c for c in args.bags[index:]]+['scale','material_box']
        try:observation=reacquire(source,vision,required,observation_root/color,on_scale)
        except VisionError as error:record_vision_failure(args.state_dir,results,error,color)
        task.set_head_command(observation['head_last_command_deg'])
        evidence_files[color]=observation['evidence_file'];atomic_json(args.state_dir/'latest_observation_files.json',evidence_files)
        scale_reader=lambda task,bag:read_scale_from_environment_rgb(task,bag,args.noise_level,args.camera_roll_deg,source.rng,conditions)
        reference=cv2.cvtColor(cv2.imread(observation['reference_rgb_file']),cv2.COLOR_BGR2RGB)
        verifier=SourceClearanceVerifier(vision,reference,observation['detections']['bag_'+color],observation['detections']) if 'bag_'+color not in on_scale else None
        def verify_source(task,bag):
            output=args.state_dir/'source_verification'/attempt_id/bag;frames=[]
            for rgb in source.frames(output):
                frames.append(rgb)
                if len(frames)<3:continue
                report=verifier.verify_sequence(frames)
                if report['verified']:break
            report=verifier.verify_sequence(frames);atomic_json(output/'result.json',report)
            if report['verified']:task.set_head_command(source.last_view)
            return report
        def observe_placement(task,bag):
            output=args.state_dir/'placement_verification'/attempt_id/bag
            updated=reacquire(source,vision,required,output,[bag])
            det=updated['detections'];yaw=det['scale'].get('yaw_table_rad',0)
            expected=np.array(det['scale']['table_xy_m'])+.06*np.array([-np.sin(yaw),np.cos(yaw)])
            error=float(np.linalg.norm(np.asarray(det[bag]['table_xy_m'])-expected)*1000)
            report={'verified':error<=15,'placement_error_mm':error,'tolerance_mm':15,
                    'expected_table_xy_m':expected.tolist(),'observed_table_xy_m':det[bag]['table_xy_m']}
            atomic_json(output/'result.json',report)
            if not report['verified']:raise VisionError('RGB parcel position does not agree with placement on scale: '+json.dumps(report))
            task.set_head_command(updated['head_last_command_deg'])
            return updated
        release_environment=lambda task,bag,record:settle_bin_environment(task.scene,task.state_dir,bag,
            task.arm._eef(record['arm'])[0]+np.asarray(record['grasp_offset_world_m']),record['action_id'])
        result=task.cycle([color],True,observation,scale_reader,verify_source if verifier else None,observe_placement,release_environment)
        entry={'bag':color,'result':result,'consensus':observation['consensus']}
        attempts.append(entry)
        results=[r for r in results if r['bag']!=color]+[entry]
        remaining=args.bags[index+1:] if result['status']=='executed' else args.bags[index:]
        overall='running' if remaining and result['status']=='executed' else result['status']
        atomic_json(args.state_dir/'closed_loop_result.json',{'results':results,'attempts':attempts,'status':overall,
                    'remaining_bags':remaining,'completed_bags':[r['bag'] for r in results if r['result'].get('status')=='executed'],
                    'attempt_id':attempt_id,'latest_observation_files':evidence_files})
        print(color,result['status'],flush=True)
        if result['status']!='executed':raise SystemExit(2)
    manifest['status']='executed';atomic_json(args.state_dir/'runs'/(attempt_id+'.json'),manifest)


if __name__=='__main__':main()
