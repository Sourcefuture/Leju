#!/usr/bin/env python3
"""Private scorer: compare RGB-planned arms/held parcels with actual scene geometry."""
import argparse,json,os
from pathlib import Path
os.environ.setdefault('MUJOCO_GL','egl')
import mujoco,numpy as np
from kuavo_planar_task import apply_objects,atomic_json
from kuavo_mujoco_live_viewer import arm_addresses
from kuavo_sim_bin_environment import apply_private_bin_poses


def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--scene',type=Path,required=True);p.add_argument('--history',type=Path,required=True)
    p.add_argument('--output',type=Path,required=True);a=p.parse_args()
    m=mujoco.MjModel.from_xml_path(str(a.scene.resolve()));d=mujoco.MjData(m)
    if m.nkey:mujoco.mj_resetDataKeyframe(m,d,0)
    addresses=arm_addresses(m)
    arms=[i for i in range(m.ngeom) if m.geom_group[i]==3 and m.body(m.geom_bodyid[i]).name.startswith('zarm_')]
    objects=[i for i in range(m.ngeom) if int(m.geom_contype[i])&8]
    records=[json.loads(line) for line in a.history.read_text().splitlines()]
    violations=[];samples=0
    for r in records:
        if r.get('kind')!='planar_task' or r.get('status')!='executed':continue
        path=np.asarray(r['trajectory_arm14_rad']);states=r.get('task_objects',{})
        held=[m.body(n).id for n,s in states.items() if s.get('attached_arm')]
        for k,(before,after) in enumerate(zip(path[:-1],path[1:])):
            for fraction in [0.,.5,1.]:
                d.qpos[addresses]=before*(1-fraction)+after*fraction;mujoco.mj_forward(m,d)
                apply_objects(m,d,states,1.,None);samples+=1
                apply_private_bin_poses(m,d,states,a.history.parent,fraction,
                    r.get('bag') if r.get('event')=='drop' else None,r.get('action_id'))
                pairs=[(robot,obj) for robot in arms for obj in objects
                       if not (m.geom_bodyid[obj] in held and m.body(m.geom_bodyid[robot]).name.endswith('7_link'))]
                pairs += [(parcel,obj) for parcel in objects if m.geom_bodyid[parcel] in held
                          for obj in objects if m.geom_bodyid[obj] not in held]
                for first,second in pairs:
                    distance=mujoco.mj_geomDistance(m,d,first,second,.01,None)
                    if distance<-.002:
                        violations.append({'stage':r['name'],'segment':k,'fraction':fraction,
                                           'first_body':m.body(m.geom_bodyid[first]).name,
                                           'first_geom':m.geom(first).name,'second_geom':m.geom(second).name,
                                           'penetration_mm':-float(distance)*1000})
    result={'passed':bool(records) and not violations,'checked_samples':samples,'violations':violations,
            'input_separation':'actual object geometry is used only by this private scorer, never by localization/planning',
            'scope':'official arm collision primitives and actual task/held-parcel primitives; assisted grasp/drop, penetration tolerance 2 mm'}
    atomic_json(a.output,result);print(json.dumps({k:v for k,v in result.items() if k!='violations'},indent=2))
    if violations:print(json.dumps(violations[:5],indent=2));raise SystemExit(2)


if __name__=='__main__':main()
