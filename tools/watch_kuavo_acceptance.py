#!/usr/bin/env python3
"""Audit/render completed local acceptance runs, with a bounded wall-time watch."""
import argparse,json,os,subprocess,sys,time
from concurrent.futures import ThreadPoolExecutor
from pathlib import Path


def main():
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('--root',type=Path,default=Path('outputs/kuavo_sim/acceptance_20261014'))
    p.add_argument('--minutes',type=float,default=120);a=p.parse_args();tools=Path(__file__).resolve().parent
    env={**os.environ,'OPENBLAS_NUM_THREADS':'1','OMP_NUM_THREADS':'1','LP_NUM_THREADS':'2','MUJOCO_GL':'egl','LIBGL_ALWAYS_SOFTWARE':'true'}
    active={};done=set();start=time.monotonic()
    def work(case):
        scene=Path(case['scene']);state=Path(case['state_dir']);out=Path(case.get('output_dir',scene.parent));out.mkdir(parents=True,exist_ok=True)
        command=[sys.executable,str(tools/'audit_kuavo_acceptance_case.py'),'--scene',str(scene),'--state-dir',str(state),'--output-dir',str(out)]
        with (out/'acceptance_worker.log').open('w') as log:run=subprocess.run(command,env=env,stdout=log,stderr=subprocess.STDOUT,timeout=1500)
        if run.returncode!=0:return {'case':case['label'],'audit_exit_code':run.returncode}
        args=[sys.executable,str(tools/'render_kuavo_planar_task.py'),'--scene',str(scene),'--state-dir',str(state),'--output',case['video'],'--fps','5','--playback-speed','2','--sensor-panel','--label',case.get('conditions',case['label'])]
        # Viewer input disturbances must match the recorded declared setup.
        runs=sorted((state/'runs').glob('*.json'),key=lambda f:f.stat().st_mtime)
        if runs:
            settings=json.loads(runs[-1].read_text())['args']
            if settings.get('view_conditions'):args.extend(['--view-conditions',settings['view_conditions']])
            args.extend(['--noise-level',str(settings.get('noise_level',0)),'--camera-roll-deg',str(settings.get('camera_roll_deg',0))])
        with (out/'render_worker.log').open('w') as log:r=subprocess.run(args,env=env,stdout=log,stderr=subprocess.STDOUT,timeout=2700)
        return {'case':case['label'],'audit_exit_code':0,'render_exit_code':r.returncode}
    with ThreadPoolExecutor(max_workers=2) as pool:
        while time.monotonic()-start<a.minutes*60:
            subprocess.run([sys.executable,str(tools/'build_kuavo_acceptance_report.py'),'--root',str(a.root)],env=env,stdout=subprocess.DEVNULL,check=False)
            summary=a.root/'cases_summary.json'
            if summary.is_file():
                for row in json.loads(summary.read_text()):
                    metadata=Path(row['video']).with_suffix('.json')
                    movie=json.loads(metadata.read_text()) if metadata.is_file() else {}
                    if row.get('verified_scoped_simulation') and row.get('video_matches_current_run') and movie.get('video_codec')=='h264' and movie.get('caption_font'):
                        done.add(row['state_dir'])
            cases=json.loads((a.root/'cases.json').read_text())['cases']
            for case in cases:
                key=case['state_dir'];state=Path(key);file=state/'closed_loop_result.json'
                if key in done or key in active or not file.is_file():continue
                result=json.loads(file.read_text())
                if result.get('status')!='executed':continue
                drops=[json.loads(p.read_text()) for p in (state/'private_bin_drops').glob('*.json')]
                if not drops or not all(d.get('private_contact_score_passed') for d in drops):continue
                if len(active)<2:active[key]=pool.submit(work,case);print('queued',case['label'],flush=True)
            for key in list(active):
                if active[key].done():
                    try:result=active.pop(key).result()
                    except Exception as e:result={'case':key,'worker_error':str(e)}
                    done.add(key);print(result,flush=True)
                    (a.root/'worker_progress.json').write_text(json.dumps({'done':sorted(done),'active':sorted(active)},indent=2)+'\n')
            subprocess.run([sys.executable,str(tools/'build_kuavo_acceptance_report.py'),'--root',str(a.root)],env=env,stdout=subprocess.DEVNULL,check=False)
            time.sleep(15)


if __name__=='__main__':main()
