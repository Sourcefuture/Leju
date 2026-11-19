#!/usr/bin/env python3
"""Check recorded current-state continuity without loading the simulator."""
import argparse, hashlib, json, math
from pathlib import Path


def check(state):
    history = state / 'history.jsonl'
    rows = [json.loads(line) for line in history.read_text().splitlines()]
    rows = [r for r in rows if r.get('status') == 'executed' and r.get('kind') == 'planar_task']
    errors = []; previous = None; maximum_seam = 0.; ids = set()
    for r in rows:
        stage = r.get('name', '?'); path = r.get('trajectory_arm14_rad', [])
        if not path or any(len(q) != 14 or not all(math.isfinite(v) for v in q) for q in path):
            errors.append({'stage': stage, 'reason': 'invalid 14D trajectory'}); continue
        target = r.get('target_arm14_rad', [])
        if len(target) != 14 or max(abs(a-b) for a,b in zip(target,path[-1])) > 1e-9:
            errors.append({'stage': stage, 'reason': 'saved target differs from trajectory end'})
        if previous is not None:
            seam = max(abs(a-b) for a,b in zip(previous,path[0])); maximum_seam = max(maximum_seam,seam)
            if seam > 1e-9: errors.append({'stage': stage, 'reason': 'does not start at previous current state', 'error_rad': seam})
        previous = path[-1]
        ident = r.get('action_id')
        if not ident or ident in ids: errors.append({'stage':stage,'reason':'missing or duplicate action ID'})
        ids.add(ident)
        times = r.get('trajectory_t_s', [])
        if len(times) != len(path) or any(b <= a for a,b in zip(times,times[1:])):
            errors.append({'stage':stage,'reason':'invalid trajectory time sequence'})
        elif abs(times[-1]-r['actual_duration_s']) > 1e-6:
            errors.append({'stage':stage,'reason':'duration does not match recorded time sequence'})
        radians = r.get('target_arm14_rad',[]); degrees = r.get('target_arm14_deg',[])
        if len(degrees) != 14 or any(abs(math.degrees(a)-b)>1e-7 for a,b in zip(radians,degrees)):
            errors.append({'stage':stage,'reason':'degrees/radians mismatch'})
    latest = json.loads((state/'latest_state.json').read_text())
    current = latest.get('arm14_rad',[])
    final_error = max(abs(a-b) for a,b in zip(previous,current)) if previous is not None and len(current)==14 else None
    if final_error is None or final_error > 1e-9:
        errors.append({'reason':'persisted current state differs from final trajectory','error_rad':final_error})
    return {'passed': bool(rows) and not errors, 'actions': len(rows),
            'maximum_state_seam_rad':maximum_seam,'persisted_final_state_error_rad':final_error,
            'history_sha256':hashlib.sha256(history.read_bytes()).hexdigest(), 'violations':errors,
            'scope':'recorded local simulation 14D trajectory continuity and persisted current state; not hardware encoder feedback'}


def main():
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('--state-dir',type=Path,required=True)
    p.add_argument('--output',type=Path,required=True);a=p.parse_args();result=check(a.state_dir)
    a.output.parent.mkdir(parents=True,exist_ok=True);a.output.write_text(json.dumps(result,indent=2)+'\n')
    print(json.dumps(result,indent=2));raise SystemExit(0 if result['passed'] else 2)


if __name__=='__main__':main()
