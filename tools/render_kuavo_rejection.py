#!/usr/bin/env python3
"""Render real failed acquisition snapshots with an unexecuted scene preview."""
import argparse,json,os,textwrap
from pathlib import Path
os.environ.setdefault('MUJOCO_GL','egl')
import cv2,numpy as np
from kuavo_sim_camera import KuavoSimCamera


def main():
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('--scene',type=Path,required=True)
    p.add_argument('--state-dir',type=Path,required=True);p.add_argument('--output',type=Path,required=True)
    p.add_argument('--label',default='Rejected experiment');p.add_argument('--fps',type=int,default=6)
    a=p.parse_args();result_file=a.state_dir/'closed_loop_result.json'
    if not result_file.is_file():result_file=a.state_dir/'closed_loop_error.json'
    report=json.loads(result_file.read_text()) if result_file.is_file() else {'status':'unknown','reason':'No completed result'}
    reason=report.get('reason') or next((r['result'].get('reason') or str(r['result'].get('failure',{})) for r in reversed(report.get('results',[])) if r['result'].get('status')!='executed'),'Inspect the per-stage failure report')
    images=sorted((a.state_dir/'observations').glob('**/*.png'))
    if not images:images=sorted((a.state_dir/'placement_verification').glob('**/*.png'))
    camera=KuavoSimCamera(a.scene,a.state_dir)
    try:world=cv2.cvtColor(camera.capture_rgb('overview'),cv2.COLOR_RGB2BGR)
    finally:camera.close()
    a.output.parent.mkdir(parents=True,exist_ok=True);writer=cv2.VideoWriter(str(a.output),cv2.VideoWriter_fourcc(*'mp4v'),a.fps,(1280,720))
    if not writer.isOpened():raise RuntimeError('Video writer failed')
    frames=0
    try:
        for filename in images[:30] or [None]:
            source=cv2.imread(str(filename)) if filename else np.zeros((480,848,3),np.uint8)
            if source is None:continue
            canvas=np.full((720,1280,3),24,np.uint8);canvas[105:558,:800]=cv2.resize(world,(800,453));canvas[105:377,800:]=cv2.resize(source,(480,272))
            texts=[(30,a.label),(65,'REJECTED / STOPPED | source RGB is shown, no fabricated successful motion'),(92,str(filename.name) if filename else 'No RGB acquired')]
            texts += [(602+i*24,line) for i,line in enumerate(textwrap.wrap(str(reason),width=155)[:4])]
            for y,text in texts:cv2.putText(canvas,text[:180],(12,y),cv2.FONT_HERSHEY_SIMPLEX,.52,(210,225,245),1)
            for _ in range(a.fps):writer.write(canvas);frames+=1
        cv2.imwrite(str(a.output.with_suffix('.png')),canvas)
    finally:writer.release()
    a.output.with_suffix('.json').write_text(json.dumps({'status':report.get('status'),'frames':frames,'fps':a.fps,'duration_s':frames/a.fps,'reason':reason,
        'scope':'failed input snapshots and fixed last-state preview; this video does not replay a successful robot operation'},indent=2)+'\n')
    print(a.output)


if __name__=='__main__':main()
