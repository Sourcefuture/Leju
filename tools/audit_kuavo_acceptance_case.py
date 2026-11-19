#!/usr/bin/env python3
"""Run independent acceptance checks on one completed local simulation case."""
import argparse,json,os,subprocess,sys
from pathlib import Path


def main():
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('--scene',type=Path,required=True)
    p.add_argument('--state-dir',type=Path,required=True);p.add_argument('--output-dir',type=Path,required=True)
    a=p.parse_args();a.output_dir.mkdir(parents=True,exist_ok=True);root=Path(__file__).resolve().parent
    env={**os.environ,'OPENBLAS_NUM_THREADS':'1','OMP_NUM_THREADS':'1'}
    commands=[('state_history',root/'check_kuavo_state_history.py',['--state-dir',str(a.state_dir),'--output',str(a.output_dir/'state_history_check.json')]),
        ('rgb_weights',root/'check_kuavo_rgb_weights.py',['--state-dir',str(a.state_dir),'--output',str(a.output_dir/'rgb_weight_check.json')]),
        ('fixture',root/'check_kuavo_fixture.py',['--scene',str(a.scene),'--output',str(a.output_dir/'fixture_check.json')]),
        ('trajectory',root/'audit_kuavo_planar_task.py',['--scene',str(a.scene),'--state-dir',str(a.state_dir),'--output',str(a.output_dir/'trajectory_audit.json')]),
        ('contacts',root/'score_kuavo_sim_contacts.py',['--scene',str(a.scene),'--history',str(a.state_dir/'history.jsonl'),'--output',str(a.output_dir/'actual_contacts.json')]),
        ('stance',root/'score_kuavo_sim_stance.py',['--scene',str(a.scene),'--history',str(a.state_dir/'history.jsonl'),'--output',str(a.output_dir/'stance_score.json')]),
        ('physical',root/'score_kuavo_physical_feasibility.py',['--scene',str(a.scene),'--history',str(a.state_dir/'history.jsonl'),'--output',str(a.output_dir/'physical_score.json')])]
    rows=[]
    for name,script,args in commands:
        print('audit',name,flush=True)
        run=subprocess.run([sys.executable,str(script),*args],env=env,text=True,capture_output=True,timeout=600)
        (a.output_dir/(name+'_audit.log')).write_text(run.stdout+run.stderr)
        rows.append({'check':name,'exit_code':run.returncode})
    result=json.loads((a.state_dir/'closed_loop_result.json').read_text());drops=[json.loads(x.read_text()) for x in (a.state_dir/'private_bin_drops').glob('*.json')]
    duplicate=set();seen=set()
    for line in (a.state_dir/'history.jsonl').read_text().splitlines():
        entry=json.loads(line)
        if entry['action_id'] in seen:duplicate.add(entry['action_id'])
        seen.add(entry['action_id'])
    drop_pass=len(drops)==len(result.get('completed_bags',[])) and all(d.get('private_contact_score_passed',False) for d in drops)
    summary={'passed':result.get('status')=='executed' and all(r['exit_code']==0 for r in rows) and drop_pass and not duplicate,
        'checks':rows,'drop_checks_passed':drop_pass,'drop_count':len(drops),'duplicate_action_ids':list(duplicate),
        'scope':'the stated local RGB/kinematics/rigid-release and necessary dynamics checks; actual finger grasp and real robot execution remain unverified'}
    (a.output_dir/'acceptance_audit.json').write_text(json.dumps(summary,indent=2)+'\n');print(json.dumps(summary,indent=2),flush=True)
    if not summary['passed']:raise SystemExit(2)


if __name__=='__main__':main()
