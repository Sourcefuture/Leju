#!/usr/bin/env python3
"""Decode every delivery video and inspect current run/evidence associations."""
import argparse,hashlib,json
from pathlib import Path
import av


def inspect_video(path,expected=None):
    count=0;bad=[]
    with av.open(str(path)) as container:
        stream=container.streams.video[0];codec=stream.codec_context.name
        rate=float(stream.average_rate);width=stream.width;height=stream.height
        for frame in container.decode(stream):
            if frame.width!=width or frame.height!=height:bad.append('frame dimensions changed')
            count+=1
    passed=count>0 and codec=='h264' and (expected is None or count==expected) and not bad
    return {'passed':passed,'decoded_frames':count,'expected_frames':expected,'codec':codec,
            'fps':rate,'width':width,'height':height,'video_sha256':hashlib.sha256(path.read_bytes()).hexdigest(),'violations':bad}


def main():
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('--root',type=Path,default=Path('outputs/kuavo_sim/acceptance_20261014'))
    a=p.parse_args();rows=json.loads((a.root/'cases_summary.json').read_text());results=[]
    for row in rows:
        movie=Path(row['video']);meta=json.loads(movie.with_suffix('.json').read_text())
        decoded=inspect_video(movie,meta.get('frames'));decoded.update(case=row['label'],video=str(movie),
            outcome=row['status'],scoped_simulation_passed=row['verified_scoped_simulation'],
            matches_recorded_run=row['video_matches_current_run'])
        state=Path(row['state_dir']);history=state/'history.jsonl'
        if row['verified_scoped_simulation']:
            records=[json.loads(line) for line in history.read_text().splitlines()]
            executed=[r for r in records if r.get('kind')=='planar_task' and r.get('status')=='executed']
            missing=[]
            for record in executed:
                observation=record.get('vision_observation_file')
                if not observation or not Path(observation).is_file():missing.append(record['name'])
                if record.get('weighing'):
                    for key in ['rgb_evidence_file','lcd_evidence_file']:
                        if not Path(record['weighing'][key]).is_file():missing.append(record['name']+':'+key)
            decoded['full_actions']=len(executed);decoded['missing_evidence']=missing
            decoded['passed'] &= len(executed)==63 and not missing and meta.get('history_sha256')==hashlib.sha256(history.read_bytes()).hexdigest()
        decoded['passed'] &= row['video_matches_current_run']
        results.append(decoded);print(row['label'],decoded['passed'],decoded['decoded_frames'],flush=True)
    counter=a.root/'deep_gap30/physics_no_grasp_rotor.mp4';physical=json.loads(counter.with_suffix('.json').read_text())
    decoded=inspect_video(counter,physical.get('encoded_frames'));decoded['counterexample_consistent']=physical['max_actual_parcel_lift_mm']==0 and not physical['grasp_succeeded']
    decoded['passed'] &= decoded['counterexample_consistent'];results.append({'case':'actual dynamics without grasp',**decoded})
    report={'passed':all(row['passed'] for row in results),'scenario_videos':len(rows),
            'complete_scoped_passes':sum(r['verified_scoped_simulation'] for r in rows),'videos':results,
            'scope':'all-frame decoding, recorded trajectory/evidence association and existing scoped gates; not actual hand/real robot validation'}
    (a.root/'delivery_artifact_check.json').write_text(json.dumps(report,ensure_ascii=False,indent=2)+'\n')
    if not report['passed']:raise SystemExit(2)


if __name__=='__main__':main()
