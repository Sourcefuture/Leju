#!/usr/bin/env python3
"""Private setup check: source fixtures must not overlap before arm motion."""
import argparse,json
from pathlib import Path
import mujoco,numpy as np


def check(scene):
    m=mujoco.MjModel.from_xml_path(str(Path(scene).resolve()));d=mujoco.MjData(m);mujoco.mj_resetDataKeyframe(m,d,0);mujoco.mj_forward(m,d)
    names=['bag_white','bag_green','bag_black','scale','material_box'];ids={name:m.body(name).id for name in names}
    geoms={name:[i for i in range(m.ngeom) if m.geom_bodyid[i]==ident and int(m.geom_contype[i])&8] for name,ident in ids.items()}
    overlaps=[];package_clearances=[]
    for index,first in enumerate(names):
        for second in names[index+1:]:
            depth=min(mujoco.mj_geomDistance(m,d,i,j,.01,None) for i in geoms[first] for j in geoms[second])
            if depth<-.002:overlaps.append({'first':first,'second':second,'penetration_mm':-float(depth)*1000})
            if first.startswith('bag_') and second.startswith('bag_'):
                separation=min(mujoco.mj_geomDistance(m,d,i,j,1.,None) for i in geoms[first] for j in geoms[second])
                package_clearances.append({'first':first,'second':second,'surface_clearance_mm':float(separation)*1000})
    table=m.body('task_table').id;r=d.xmat[table].reshape(3,3);edge=d.xpos[table]+r@np.array([.6,0,0])
    feet=np.mean([d.body('leg_'+s+'6_link').xpos for s in ['l','r']],axis=0)
    centres={name:(r.T@(d.xpos[body]-d.xpos[table]))[:2].tolist() for name,body in ids.items()}
    outside=[name for name,xy in centres.items() if min(xy)<0 or xy[0]>1.2 or xy[1]>.6]
    initial_joint_limits=[]
    for joint in range(m.njnt):
        if not m.jnt_limited[joint]:continue
        value=float(d.qpos[m.jnt_qposadr[joint]]);lo,hi=m.jnt_range[joint]
        if not lo-1e-7<=value<=hi+1e-7:
            initial_joint_limits.append({'joint':m.joint(joint).name,'position_rad':value,'limits_rad':[float(lo),float(hi)]})
    return {'source_fixture_passed':not overlaps and not outside and not initial_joint_limits,'overlaps':overlaps,'unsupported_centres':outside,
        'initial_whole_robot_joint_limits_passed':not initial_joint_limits,'initial_joint_limit_violations':initial_joint_limits,
        'source_package_clearances':package_clearances,
        'minimum_source_package_clearance_mm':min(x['surface_clearance_mm'] for x in package_clearances),
        'feet_to_near_edge_perpendicular_m':float(r[:,1]@(edge-feet)),
        'pelvis_to_near_edge_perpendicular_m':float(r[:,1]@(edge-d.qpos[:3])),
        'scope':'source rigid CAD fixtures and model joint limits of the fixed preparation; no actuator, stance-transition, grasp or soft-material check'}


def main():
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('--scene',type=Path,required=True);p.add_argument('--output',type=Path,required=True)
    a=p.parse_args();r=check(a.scene);a.output.parent.mkdir(parents=True,exist_ok=True);a.output.write_text(json.dumps(r,indent=2)+'\n');print(json.dumps(r,indent=2))
    if not r['source_fixture_passed']:raise SystemExit(2)


if __name__=='__main__':main()
