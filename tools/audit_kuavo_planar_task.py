#!/usr/bin/env python3
"""Independent FK, collision and interpolation audit of recorded simulation arms."""
import argparse,json,os
from pathlib import Path
os.environ.setdefault('MUJOCO_GL','egl')
import mujoco,numpy as np
from kuavo_planar_task import PlanarTask,atomic_json,CONTACT_OFFSETS


def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--scene',type=Path,required=True);p.add_argument('--state-dir',type=Path,required=True)
    p.add_argument('--observation',type=Path);p.add_argument('--output',type=Path,required=True)
    a=p.parse_args();task=PlanarTask(a.scene,a.state_dir)
    if a.observation is None:
        index=a.state_dir/'latest_observation_files.json'
        a.observation=Path(json.loads(index.read_text())['initial']) if index.is_file() else a.state_dir/'observations/initial/observation.json'
    task.set_collision_world(json.loads(a.observation.read_text()));world=task.collision_world
    records=[json.loads(line) for line in (a.state_dir/'history.jsonl').read_text().splitlines()]
    records=[r for r in records if r.get('kind')=='planar_task' and r.get('status')=='executed']
    violations=[];max_angle=0.;max_end_error=0.;max_speed=0.;samples=0
    previous_bag=None
    for record in records:
        packet=record.get('bag','').removeprefix('bag_')
        observed=Path(record['vision_observation_file']) if record.get('vision_observation_file') else a.state_dir/'observations'/packet/'observation.json'
        key=str(observed)
        if key!=previous_bag and observed.is_file():
            task.set_collision_world(json.loads(observed.read_text()));world=task.collision_world
        previous_bag=key
        objects=record.get('task_objects',{})
        world.set_held()
        for name,obj in objects.items():
            if obj.get('attached_arm'):
                world.set_held(name,obj['attached_arm'],obj.get('tcp_local_m',CONTACT_OFFSETS[obj['attached_arm']]),obj['offset_world_m'],record['trajectory_arm14_rad'][0])
            elif obj.get('world_xyz_m'):world.relocate_bag(name,obj['world_xyz_m'])
        packed=sum(o.get('destination')=='bin' and 'drop_from_world_xyz_m' not in o for o in objects.values())
        world.set_packed_count(packed)
        path=np.asarray(record['trajectory_arm14_rad'])
        for index,(before,after) in enumerate(zip(path[:-1],path[1:])):
            for fraction in [0.,.25,.5,.75,1.]:
                q=before*(1-fraction)+after*fraction
                task.arm.data.qpos[task.arm.qpos_addresses]=q;mujoco.mj_forward(task.arm.model,task.arm.data)
                collisions=world.check(q)
                angles=[]
                for side in ['left','right']:
                    error=np.zeros(3);mujoco.mju_subQuat(error,task.down[side],task.arm._eef(side)[2])
                    angles.append(float(np.degrees(np.linalg.norm(error))))
                max_angle=max(max_angle,*angles);samples+=1
                if collisions or max(angles)>1:
                    violations.append({'stage':record['name'],'segment':index,'fraction':fraction,
                                       'orientation_error_deg':angles,'collisions':collisions})
        durations=np.diff(record['trajectory_t_s']) if record.get('trajectory_t_s') else np.full(len(path)-1,record['actual_duration_s']/(len(path)-1))
        speed=float(np.max(np.abs(np.degrees(np.diff(path,axis=0)))/durations[:,None]))
        if record.get('trajectory_profile')=='quintic_stop_at_waypoints':speed*=1.875
        max_speed=max(max_speed,speed)
        task.arm.data.qpos[task.arm.qpos_addresses]=path[-1];mujoco.mj_forward(task.arm.model,task.arm.data)
        error=float(np.linalg.norm(task.arm._eef(record['arm'])[0]-record['requested_xyz_m'])*1000)
        max_end_error=max(max_end_error,error)
        if error>3 or speed>25.00001:violations.append({'stage':record['name'],'endpoint_error_mm':error,'speed_deg_s':speed})
    result={'passed':bool(records) and not violations,'actions':len(records),'checked_samples':samples,
            'max_orientation_error_deg':max_angle,'max_endpoint_error_mm':max_end_error,
            'max_joint_speed_deg_s':max_speed,'violations':violations,
            'scope':'CAD arm/torso/cross-arm/held-parcel against RGB-reconstructed obstacles; assisted grasp, kinematic drop, no whole-body dynamics'}
    atomic_json(a.output,result);print(json.dumps({k:v for k,v in result.items() if k!='violations'},indent=2))
    if violations:print(json.dumps(violations[:3],indent=2));raise SystemExit(2)


if __name__=='__main__':main()
