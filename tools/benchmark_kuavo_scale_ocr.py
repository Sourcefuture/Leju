#!/usr/bin/env python3
"""Score random displayed masses through disturbed robot head RGB images."""
import argparse,json,os
from pathlib import Path
os.environ.setdefault('MUJOCO_GL','egl')
import cv2,mujoco,numpy as np
from kuavo_sim_camera import KuavoSimCamera
from kuavo_planar_task import apply_objects
from kuavo_scale_vision import write_lcd_texture,read_scale_sequence
from benchmark_kuavo_table_vision import perturb_rgb


def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--scene',type=Path,required=True);p.add_argument('--history',type=Path,required=True)
    p.add_argument('--output',type=Path,required=True);p.add_argument('--trials',type=int,default=100)
    p.add_argument('--seed',type=int,default=55);p.add_argument('--max-frames',type=int,default=10)
    a=p.parse_args();a.output.mkdir(parents=True,exist_ok=True)
    records=[json.loads(line) for line in a.history.read_text().splitlines()]
    weighing=next(r for r in records if r.get('event')=='weigh')
    reader=KuavoSimCamera(a.scene,a.output/'unused_state',1280,720)
    reader.data.qpos[reader.addresses]=weighing['target_arm14_rad'];mujoco.mj_forward(reader.model,reader.data)
    apply_objects(reader.model,reader.data,weighing['task_objects'])
    rng=np.random.default_rng(a.seed);rows=[]
    try:
        for trial in range(a.trials):
            expected=int(rng.integers(1,50000))/10000
            ident=write_lcd_texture(reader.model,expected)
            reader.renderer._gl_context.make_current();mujoco.mjr_uploadTexture(reader.model,reader.renderer._mjr_context,ident)
            frames=[];reading=None;reason='no agreeing RGB frames'
            for index in range(a.max_frames):
                reader.data.qpos[reader.model.joint('zhead_1_joint').qposadr[0]]=np.radians(rng.uniform(-6,6))
                reader.data.qpos[reader.model.joint('zhead_2_joint').qposadr[0]]=np.radians(rng.uniform(10,20))
                mujoco.mj_forward(reader.model,reader.data)
                rgb,_,_=reader.capture('head');frames.append(perturb_rgb(rgb,rng,2))
                if len(frames)>=3:
                    try:reading,lcd=read_scale_sequence(frames,minimum=3);break
                    except ValueError as error:reason=str(error)
            row={'trial':trial,'private_expected_kg':expected}
            if reading:
                row.update(status='read',reading=reading,error_g=abs(reading['measured_kg']-expected)*1000)
            else:row.update(status='rejected',reason=reason)
            row['frames']=len(frames)
            if row['status']=='rejected' or row.get('error_g',0)>.15:
                cv2.imwrite(str(a.output/f'{trial:03d}_rgb.png'),cv2.cvtColor(frames[0],cv2.COLOR_RGB2BGR))
            rows.append(row)
            (a.output/'partial.json').write_text(json.dumps(rows,indent=2))
            if trial%10==0:print(trial+1,row['status'],row.get('error_g'),flush=True)
    finally:reader.close()
    reads=[r for r in rows if r['status']=='read']
    metrics={'trials':len(rows),'read':len(reads),'correct_within_0_15g':sum(r['error_g']<=.15 for r in reads),
             'false_read':sum(r['error_g']>.15 for r in reads),'rejects':len(rows)-len(reads),'seed':a.seed,
             'max_frames':a.max_frames,'mean_frames':float(np.mean([r['frames'] for r in rows])),
             'head_yaw_deg':[-6,6],'head_pitch_deg':[10,20],'noise_level':2,
             'input_contract':'decoder receives only RGB; displayed mass private to scorer'}
    (a.output/'report.json').write_text(json.dumps({'metrics':metrics,'records':rows},indent=2));print(json.dumps(metrics,indent=2))


if __name__=='__main__':main()
