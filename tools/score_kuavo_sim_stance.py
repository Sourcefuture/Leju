#!/usr/bin/env python3
"""Necessary quasi-static stance checks; this does not validate whole-body control."""
import argparse,json,os
from pathlib import Path
os.environ.setdefault('MUJOCO_GL','egl')
import cv2,mujoco,numpy as np
from kuavo_planar_task import apply_objects,atomic_json
from kuavo_mujoco_live_viewer import arm_addresses


def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--scene',type=Path,required=True);p.add_argument('--history',type=Path,required=True)
    p.add_argument('--output',type=Path,required=True);a=p.parse_args()
    m=mujoco.MjModel.from_xml_path(str(a.scene.resolve()));d=mujoco.MjData(m)
    mujoco.mj_resetDataKeyframe(m,d,0);mujoco.mj_forward(m,d)
    root=m.body('base_link').id;robot_mass=float(m.body_subtreemass[root]);addresses=arm_addresses(m)
    floor=m.geom('tabletop_floor').id;foot_geoms=[i for i in range(m.ngeom) if m.geom_group[i]==3
                 and m.body(m.geom_bodyid[i]).name in ['leg_l6_link','leg_r6_link']]
    points=[]
    for ident in foot_geoms:
        if mujoco.mj_geomDistance(m,d,ident,floor,1.,None)>.0005:continue
        rotation=d.geom_xmat[ident].reshape(3,3);size=m.geom_size[ident];kind=m.geom_type[ident]
        center=d.geom_xpos[ident]
        if kind==mujoco.mjtGeom.mjGEOM_BOX:
            corners=np.array([center+rotation@(size*np.array([x,y,z])) for x in [-1,1] for y in [-1,1] for z in [-1,1]])
            points.extend(corners[corners[:,2]<=corners[:,2].min()+.0005,:2])
        elif kind==mujoco.mjtGeom.mjGEOM_CAPSULE:
            axis=rotation[:,2];ends=[center-axis*size[1],center+axis*size[1]]
            points.extend([point[:2] for point in ends if point[2]<=min(p[2] for p in ends)+.0005])
        else:points.append(center[:2])
    polygon=cv2.convexHull(np.asarray(points,np.float32))
    loads_file=a.scene.parent/'environment_loads.json'
    loads=json.loads(loads_file.read_text()) if loads_file.is_file() else {}
    records=[json.loads(line) for line in a.history.read_text().splitlines()]
    minimum=float('inf');worst=None;samples=0
    yadr=m.joint('zhead_1_joint').qposadr[0];padr=m.joint('zhead_2_joint').qposadr[0]
    for record in records:
        if record.get('kind')!='planar_task' or record.get('status')!='executed':continue
        for q in record['trajectory_arm14_rad']:
            for yaw,pitch in [(0,0),(-15,5),(15,5),(-15,20),(15,20)]:
                d.qpos[addresses]=q;d.qpos[yadr]=np.radians(yaw);d.qpos[padr]=np.radians(pitch)
                mujoco.mj_forward(m,d);apply_objects(m,d,record.get('task_objects',{}))
                center=d.subtree_com[root].copy()*robot_mass;mass=robot_mass
                for name,obj in record.get('task_objects',{}).items():
                    if obj.get('attached_arm'):
                        load=float(loads[name]);mass+=load;center+=d.body(name).xpos*load
                center/=mass
                margin=cv2.pointPolygonTest(polygon,tuple(map(float,center[:2])),True)
                samples+=1
                if margin<minimum:minimum=margin;worst={'stage':record['name'],'com_world_xyz_m':center.tolist(),'head_yaw_pitch_deg':[yaw,pitch]}
    clearance=min(mujoco.mj_geomDistance(m,d,i,floor,1.,None) for i in foot_geoms)
    result={'necessary_checks_passed':bool(records) and minimum>.01 and abs(clearance)<.001,
            'minimum_com_projection_margin_mm':minimum*1000,'min_foot_ground_distance_mm':float(clearance)*1000,
            'robot_mass_kg':robot_mass,'samples':samples,'worst':worst,'support_polygon_xy_m':polygon[:,0].tolist(),
            'scope':'COM projection inside actual ground-touching primitive contact polygon with payload and head-view variation; no dynamic stability, WBC, friction or actuator-torque validation'}
    atomic_json(a.output,result);print(json.dumps(result,indent=2))


if __name__=='__main__':main()
