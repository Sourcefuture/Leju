#!/usr/bin/env python3
"""Isolated simulator scorer: perturb camera/rendering, feed RGB-only vision."""
from __future__ import annotations
import argparse,json,os,time
from pathlib import Path
os.environ.setdefault('MUJOCO_GL','egl')
import cv2,mujoco,numpy as np

from kuavo_sim_camera import KuavoSimCamera
from kuavo_table_vision import FourPointCalibration,TableVision,VisionError
from kuavo_rgb_disturbance import perturb_rgb

ROOT=Path(__file__).resolve().parents[1]


def orient_camera(model, ident, position, target, roll):
    z=np.asarray(position)-target;z/=np.linalg.norm(z)
    x=np.cross([0,0,1],z)
    if np.linalg.norm(x)<1e-6:x=np.array([0,-1,0.])
    x/=np.linalg.norm(x);y=np.cross(z,x)
    angle=np.radians(roll)
    right=x*np.cos(angle)+y*np.sin(angle);up=-x*np.sin(angle)+y*np.cos(angle)
    matrix=np.c_[right,up,z];quat=np.zeros(4)
    mujoco.mju_mat2Quat(quat,matrix.ravel())
    model.cam_pos[ident]=position;model.cam_quat[ident]=quat


def run(args):
    output=args.output.resolve();output.mkdir(parents=True,exist_ok=True)
    reader=KuavoSimCamera(args.scene,output/'empty_state',args.width,args.height)
    info=json.loads((args.scene.parent/'scene_info.json').read_text())
    # These four points are the explicitly permitted assumed manual input.
    corners={k[0]:p['world_xyz_m'] for k,p in info['calibration_points'].items() if k[0] in 'ABCD'}
    vision=TableVision(FourPointCalibration(corners),allow_object_tags=not args.no_object_tags)
    rng=np.random.default_rng(args.seed);records=[];ident=reader.model.camera('weighing_top').id
    head_ident=reader.model.camera('sim_head').id
    head_quat=reader.model.cam_quat[head_ident].copy()
    target=np.asarray(info['T_world_table'])[:3,3]+np.array([.3,-.6,0])
    started=time.monotonic()
    try:
        for i in range(args.trials):
            angle=rng.uniform(5,args.max_tilt);azimuth=rng.uniform(180-args.azimuth_span,180+args.azimuth_span);distance=rng.uniform(args.min_distance,args.max_distance)
            r=np.radians(angle);a=np.radians(azimuth)
            delta=distance*np.array([np.sin(r)*np.cos(a),np.sin(r)*np.sin(a),np.cos(r)])
            roll=rng.uniform(-args.max_roll,args.max_roll)
            if args.camera=='head':
                head_yaw=rng.uniform(-args.azimuth_span,args.azimuth_span)
                reader.data.qpos[reader.model.joint('zhead_1_joint').qposadr[0]]=np.radians(head_yaw)
                reader.data.qpos[reader.model.joint('zhead_2_joint').qposadr[0]]=np.radians(angle)
                roll_quat=np.array([np.cos(np.radians(roll)/2),0,0,np.sin(np.radians(roll)/2)])
                mujoco.mju_mulQuat(reader.model.cam_quat[head_ident],head_quat,roll_quat)
            else:
                orient_camera(reader.model,ident,target+delta,target,roll)
                reader.model.cam_fovy[ident]=args.fovy
            mujoco.mj_forward(reader.model,reader.data)
            rgb,_,_=reader.capture(args.camera)
            rgb=perturb_rgb(rgb,rng,args.noise_level)
            record={'trial':i,'camera_tilt_deg':angle,'camera_roll_deg':roll}
            if args.camera=='head':record.update(head_pitch_deg=angle,head_yaw_deg=head_yaw,camera_tilt_deg=30-angle)
            try:
                prediction=vision.detect(rgb)
                errors={}
                inverse=np.linalg.inv(np.asarray(info['T_world_table']))
                for name,p in prediction['detections'].items():
                    truth=(inverse@np.r_[reader.data.body(name).xpos,1])[:2]
                    errors[name]=float(np.linalg.norm(np.asarray(p['table_xy_m'])-truth)*1000)
                record.update(status='localized',errors_mm=errors,max_error_mm=max(errors.values()),
                              table_mode=prediction['quality']['mode'],
                              visible_tags=prediction['quality']['visible_ids'],
                              sources={k:p['source'] for k,p in prediction['detections'].items()})
            except VisionError as e:
                record.update(status='rejected',reason=str(e))
            records.append(record)
            if record['status']=='rejected' or record.get('max_error_mm',0)>15:
                if i<100 or args.save_all or record.get('max_error_mm',0)>15:
                    cv2.imwrite(str(output/f'case_{i:04d}.png'),cv2.cvtColor(rgb,cv2.COLOR_RGB2BGR))
            if i%10==0:
                (output/'partial.json').write_text(json.dumps(records,indent=2))
                print(f'{i+1}/{args.trials}: {record["status"]}',flush=True)
    finally:reader.close()
    successful=[r for r in records if r['status']=='localized']
    accurate=[r for r in successful if r['max_error_mm']<=15]
    metrics={'trials':len(records),'localized':len(successful),'within_15mm':len(accurate),
             'rejects':len(records)-len(successful),'elapsed_s':time.monotonic()-started,
             'seed':args.seed,'max_tilt_deg':args.max_tilt,'max_roll_deg':args.max_roll,
             'noise_level':args.noise_level,'object_tags':not args.no_object_tags,
             'fovy_deg':args.fovy,'azimuth_span_deg':args.azimuth_span,
             'camera':args.camera,
             'head_pitch_range_deg':[5,args.max_tilt] if args.camera=='head' else None,
             'distance_range_m':[args.min_distance,args.max_distance],
             'input_contract':'four assumed manual robot corner XYZ + RGB + physical marker/dimension conventions; truth scoring only',
             'per_object_p95_mm':{name:float(np.percentile([r['errors_mm'][name] for r in successful],95))
                                 for name in successful[0]['errors_mm']} if successful else {}}
    (output/'report.json').write_text(json.dumps({'metrics':metrics,'records':records},indent=2))
    print(json.dumps(metrics,indent=2))


def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--scene',type=Path,default=ROOT/'outputs/kuavo_sim/optimization/marked_scene/scene.xml')
    p.add_argument('--output',type=Path,default=ROOT/'outputs/kuavo_sim/optimization/vision_round1')
    p.add_argument('--trials',type=int,default=50);p.add_argument('--seed',type=int,default=13)
    p.add_argument('--max-tilt',type=float,default=25);p.add_argument('--max-roll',type=float,default=20)
    p.add_argument('--noise-level',type=int,default=0);p.add_argument('--no-object-tags',action='store_true')
    p.add_argument('--save-all',action='store_true');p.add_argument('--width',type=int,default=848);p.add_argument('--height',type=int,default=480)
    p.add_argument('--fovy',type=float,default=60);p.add_argument('--azimuth-span',type=float,default=25)
    p.add_argument('--min-distance',type=float,default=1.3);p.add_argument('--max-distance',type=float,default=1.8)
    p.add_argument('--camera',choices=['top','head'],default='top')
    run(p.parse_args())


if __name__=='__main__':main()
