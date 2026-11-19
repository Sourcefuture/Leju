#!/usr/bin/env python3
"""Evaluate image-only multi-view reacquisition; no truth is given to vision."""
from __future__ import annotations
import argparse,json,os,time
from pathlib import Path
os.environ.setdefault('MUJOCO_GL','egl')
import cv2,mujoco,numpy as np
from kuavo_sim_camera import KuavoSimCamera
from kuavo_table_vision import TableVision,FourPointCalibration,VisionError
from benchmark_kuavo_table_vision import perturb_rgb


def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--scene',type=Path,default=Path('outputs/kuavo_sim/optimization/marked_scene/scene.xml'))
    p.add_argument('--output',type=Path,default=Path('outputs/kuavo_sim/optimization/consensus_round1'))
    p.add_argument('--trials',type=int,default=100);p.add_argument('--seed',type=int,default=31)
    p.add_argument('--max-frames',type=int,default=10);p.add_argument('--noise-level',type=int,default=2)
    p.add_argument('--yaw-span',type=float,default=12);p.add_argument('--pitch-max',type=float,default=25)
    p.add_argument('--roll-span',type=float,default=20)
    args=p.parse_args();args.output.mkdir(parents=True,exist_ok=True)
    info=json.loads((args.scene.parent/'scene_info.json').read_text())
    corners={k[0]:v['world_xyz_m'] for k,v in info['calibration_points'].items() if k[0] in 'ABCD'}
    vision=TableVision(FourPointCalibration(corners))
    reader=KuavoSimCamera(args.scene,args.output/'empty_state')
    rng=np.random.default_rng(args.seed);rows=[]
    yawadr=reader.model.joint('zhead_1_joint').qposadr[0];pitchadr=reader.model.joint('zhead_2_joint').qposadr[0]
    cam=reader.model.camera('sim_head').id;original=reader.model.cam_quat[cam].copy()
    inverse=np.linalg.inv(np.asarray(info['T_world_table']))
    started=time.monotonic()
    try:
        for trial in range(args.trials):
            frames=[];result=None;why='not enough views'
            for n in range(args.max_frames):
                reader.data.qpos[yawadr]=np.radians(rng.uniform(-args.yaw_span,args.yaw_span))
                reader.data.qpos[pitchadr]=np.radians(rng.uniform(5,args.pitch_max))
                a=np.radians(rng.uniform(-args.roll_span,args.roll_span))/2
                mujoco.mju_mulQuat(reader.model.cam_quat[cam],original,np.array([np.cos(a),0,0,np.sin(a)]))
                mujoco.mj_forward(reader.model,reader.data)
                rgb=reader.capture_rgb('head');rgb=perturb_rgb(rgb,rng,args.noise_level)
                frames.append(rgb)
                if len(frames)>=3:
                    try:
                        result=vision.detect_sequence(frames,minimum=3,agreement_m=.015)
                        break
                    except VisionError as error:why=str(error)
            row={'trial':trial,'frames':len(frames),'status':'localized' if result else 'rejected'}
            if result:
                errors={}
                for name,det in result['detections'].items():
                    truth=(inverse@np.r_[reader.data.body(name).xpos,1])[:2]
                    errors[name]=float(np.linalg.norm(np.array(det['table_xy_m'])-truth)*1000)
                row.update(errors_mm=errors,max_error_mm=max(errors.values()),consensus=result['consensus'])
            else:row['reason']=why
            rows.append(row)
            if trial%5==0:
                (args.output/'partial.json').write_text(json.dumps(rows,indent=2))
                print(f'{trial+1}/{args.trials}: {row["status"]} ({len(frames)} frames)',flush=True)
    finally:reader.close()
    accepted=[x for x in rows if x['status']=='localized'];correct=[x for x in accepted if x['max_error_mm']<=15]
    metrics={'trials':args.trials,'localized':len(accepted),'within_15mm':len(correct),
             'rejects':args.trials-len(accepted),'false_accept_over_15mm':len(accepted)-len(correct),
             'mean_frames':float(np.mean([r['frames'] for r in rows])),'elapsed_s':time.monotonic()-started,
             'noise_level':args.noise_level,'head_yaw_deg':[-args.yaw_span,args.yaw_span],
             'head_pitch_deg':[5,args.pitch_max],'optical_roll_deg':[-args.roll_span,args.roll_span],
             'per_object_p95_mm':{name:float(np.percentile([r['errors_mm'][name] for r in accepted],95))
                                   for name in accepted[0]['errors_mm']} if accepted else {}}
    (args.output/'report.json').write_text(json.dumps({'metrics':metrics,'records':rows},indent=2))
    print(json.dumps(metrics,indent=2),flush=True)


if __name__=='__main__':main()
