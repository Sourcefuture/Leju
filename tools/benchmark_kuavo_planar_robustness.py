#!/usr/bin/env python3
"""Private scorer for table displacement, item displacement and RGB interference."""
from __future__ import annotations
import argparse,json,os,time
from pathlib import Path
os.environ.setdefault('MUJOCO_GL','egl')
import cv2,mujoco,numpy as np
from kuavo_sim_camera import KuavoSimCamera
from kuavo_table_vision import FourPointCalibration,TableVision,VisionError
from benchmark_kuavo_table_vision import perturb_rgb


def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--scene',type=Path,default=Path('outputs/kuavo_sim/optimization/marked_scene/scene.xml'))
    p.add_argument('--output',type=Path,required=True)
    p.add_argument('--trials',type=int,default=100);p.add_argument('--seed',type=int,default=72)
    p.add_argument('--calibration-noise-mm',type=float,default=2)
    p.add_argument('--occlusion-fraction',type=float,default=.025)
    p.add_argument('--radial-k1',type=float,default=.02)
    p.add_argument('--max-frames',type=int,default=30)
    a=p.parse_args();a.output.mkdir(parents=True,exist_ok=True)
    reader=KuavoSimCamera(a.scene,a.output/'unused_state');rng=np.random.default_rng(a.seed)
    model=reader.model;data=reader.data
    pristine_pos=model.body_pos.copy();pristine_quat=model.body_quat.copy()
    table=model.body('task_table').id;head=model.camera('sim_head').id
    original_head=model.cam_quat[head].copy();rows=[];started=time.monotonic()
    try:
        for trial in range(a.trials):
            model.body_pos[:]=pristine_pos;model.body_quat[:]=pristine_quat
            model.body_pos[table,:2]+=rng.uniform(-.02,.02,2)
            yaw=np.radians(rng.uniform(-5,5));q=np.array([np.cos(yaw/2),0,0,np.sin(yaw/2)])
            mujoco.mju_mulQuat(model.body_quat[table],q,pristine_quat[table])
            for name in ['bag_white','bag_green','bag_black','scale','material_box']:
                ident=model.body(name).id
                model.body_pos[ident,0]+=rng.uniform(-.02,.02)
                if not name.startswith('bag_'):model.body_pos[ident,1]+=rng.uniform(-.015,.015)
            mujoco.mj_forward(model,data)
            transform=np.eye(4);transform[:3,:3]=data.xmat[table].reshape(3,3);transform[:3,3]=data.xpos[table]
            corners={k:(transform@np.array([*xy,0,1]))[:3]+rng.uniform(-a.calibration_noise_mm,a.calibration_noise_mm,3)/1000
                     for k,xy in zip('ABCD',[[0,0],[1.2,0],[1.2,.6],[0,.6]])}
            vision=TableVision(FourPointCalibration(corners));frames=[];prediction=None;reason=''
            for index in range(a.max_frames):
                data.qpos[model.joint('zhead_1_joint').qposadr[0]]=np.radians(rng.uniform(-12,12))
                data.qpos[model.joint('zhead_2_joint').qposadr[0]]=np.radians(rng.uniform(-5,25))
                roll=np.radians(rng.uniform(-20,20))
                mujoco.mju_mulQuat(model.cam_quat[head],original_head,np.array([np.cos(roll/2),0,0,np.sin(roll/2)]))
                mujoco.mj_forward(model,data);rgb=reader.capture_rgb('head')
                rgb=perturb_rgb(rgb,rng,2)
                h,w=rgb.shape[:2];u,v=np.meshgrid(np.arange(w,dtype=np.float32),np.arange(h,dtype=np.float32))
                xn=(u-w/2)/w;yn=(v-h/2)/w
                k=rng.uniform(-a.radial_k1,a.radial_k1);s=1+k*(xn*xn+yn*yn)
                rgb=cv2.remap(rgb,w/2+xn*w*s,h/2+yn*w*s,cv2.INTER_LINEAR,borderMode=cv2.BORDER_REFLECT)
                edge=int(np.sqrt(h*w*a.occlusion_fraction))
                if edge:
                    x=int(rng.integers(0,w-edge));y=int(rng.integers(0,h-edge))
                    rgb[y:y+edge,x:x+edge]=rng.integers(20,230,3,dtype=np.uint8)
                frames.append(rgb)
                if len(frames)<3:continue
                try:prediction=vision.detect_sequence(frames,minimum=3,agreement_m=.015);break
                except VisionError as error:reason=str(error)
            row={'trial':trial,'frames':len(frames),'status':'localized' if prediction else 'rejected'}
            if prediction:
                inv=np.linalg.inv(transform);xy_errors={};robot_errors={}
                for name,det in prediction['detections'].items():
                    base=data.body(name).xpos.copy();truth=(inv@np.r_[base,1])[:2]
                    xy_errors[name]=float(np.linalg.norm(truth-det['table_xy_m'])*1000)
                    tcp=base+transform[:3,2]*det['height_m']
                    robot_errors[name]=float(np.linalg.norm(tcp-det['robot_xyz_m'])*1000)
                row.update(errors_mm=xy_errors,robot_errors_mm=robot_errors,max_error_mm=max(robot_errors.values()))
            else:row['reason']=reason
            if row['status']=='rejected' or row.get('max_error_mm',0)>15:
                cv2.imwrite(str(a.output/f'{trial:03d}_last_rgb.png'),cv2.cvtColor(frames[-1],cv2.COLOR_RGB2BGR))
            rows.append(row)
            (a.output/'partial.json').write_text(json.dumps(rows,indent=2))
            print(trial+1,row['status'],row.get('max_error_mm'),flush=True)
    finally:reader.close()
    accepted=[r for r in rows if r['status']=='localized']
    metrics={'trials':len(rows),'localized':len(accepted),'within_15mm':sum(r['max_error_mm']<=15 for r in accepted),
             'false_accept_over_15mm':sum(r['max_error_mm']>15 for r in accepted),
             'rejects':len(rows)-len(accepted),'mean_frames':float(np.mean([r['frames'] for r in rows])),
             'table_translation_mm':20,'table_yaw_deg':5,'item_translation_x_mm':20,'item_translation_y_mm':15,
             'calibration_noise_mm':a.calibration_noise_mm,'occlusion_fraction':a.occlusion_fraction,
             'max_frames_per_sequence':a.max_frames,
             'radial_k1':a.radial_k1,'elapsed_s':time.monotonic()-started,'seed':a.seed}
    (a.output/'report.json').write_text(json.dumps({'metrics':metrics,'records':rows},indent=2))
    print(json.dumps(metrics,indent=2))


if __name__=='__main__':main()
