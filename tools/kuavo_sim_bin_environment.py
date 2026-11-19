#!/usr/bin/env python3
"""Private rigid-box release settling against static filler/earlier parcels.

Physics poses are consumed only by rendering and private scoring. Localization
and IK must not read this module's pose files. This is not soft/particle physics.
"""
from __future__ import annotations
import copy,json,xml.etree.ElementTree as ET
from pathlib import Path
import mujoco,numpy as np


def read_json(path,default):
    return json.loads(Path(path).read_text()) if Path(path).is_file() else default


def write_json(path,value):
    path=Path(path);path.parent.mkdir(parents=True,exist_ok=True)
    temp=path.with_suffix('.tmp');temp.write_text(json.dumps(value,indent=2)+'\n');temp.replace(path)


def settle(scene,state_dir,bag,release_world_xyz,action_id):
    scene=Path(scene).resolve();state_dir=Path(state_dir).resolve()
    source=ET.parse(scene).getroot();table=copy.deepcopy(source.find(".//body[@name='task_table']"))
    original=mujoco.MjModel.from_xml_path(str(scene));od=mujoco.MjData(original);mujoco.mj_resetDataKeyframe(original,od,0);mujoco.mj_forward(original,od)
    shapes={name:copy.deepcopy(source.find(f".//body[@name='{name}']")) for name in ['bag_white','bag_green','bag_black']}
    for name,body in shapes.items():
        # Thin 4 mm visual layers can be crossed in one contact step. Use one
        # solid 12 mm collision slab while retaining the three visible layers.
        for geom in body.findall('geom'):
            geom.set('contype','0');geom.set('conaffinity','0');geom.attrib.pop('mass',None);geom.set('density','0')
        ET.SubElement(body,'geom',name=name+'_solid_contact',type='box',pos='0 0 .006',size='.15 .1 .006',
            contype='8',conaffinity='8',rgba='0 0 0 0',friction='.8 .01 .001',
            solref='.004 1',solimp='.95 .99 .001',density='0')
    for parent in table.iter('body'):
        for child in list(parent):
            if child.tag=='geom' and (child.get('name','').startswith('tag_') or child.get('name')=='scale_display'):parent.remove(child)
            elif child.tag=='site':parent.remove(child)
    for child in list(table):
        if child.tag=='body' and child.get('name')!='material_box':table.remove(child)
    for geom in table.iter('geom'):
        geom.attrib.pop('material',None)
        if geom.get('name','').startswith('filler_'):geom.set('contype','8');geom.set('conaffinity','8')
    root=ET.Element('mujoco',model='isolated bin release with rigid static filler')
    ET.SubElement(root,'option',timestep='.001',gravity='0 0 -9.81',iterations='100')
    world=ET.SubElement(root,'worldbody');world.append(table)
    previous=read_json(state_dir/'private_bin_environment.json',{})
    for name,pose in previous.items():
        body=shapes[name];body.set('pos',' '.join(map(str,pose['world_xyz_m'])));body.set('quat',' '.join(map(str,pose['world_quat_wxyz'])))
        world.append(body)
    parcel=shapes[bag];parcel.set('pos',' '.join(map(str,release_world_xyz)));parcel.set('quat',' '.join(map(str,od.body(bag).xquat)))
    ET.SubElement(parcel,'freejoint',name='released_parcel_free')
    load=read_json(scene.parent/'environment_loads.json',{}).get(bag,.3)
    for geom in parcel.findall('geom'):
        geom.attrib.pop('material',None)
        if geom.get('name','').endswith('_solid_contact'):geom.attrib.pop('density',None);geom.set('mass',str(float(load)))
        else:geom.set('density','0')
    world.append(parcel);m=mujoco.MjModel.from_xml_string(ET.tostring(root,encoding='unicode'));d=mujoco.MjData(m)
    mujoco.mj_forward(m,d);body=m.body(bag).id;trace=[]
    for step in range(3001):
        if step%50==0:trace.append({'t_s':float(d.time),'world_xyz_m':d.xpos[body].tolist(),'world_quat_wxyz':d.xquat[body].tolist()})
        mujoco.mj_step(m,d)
    container=m.body('material_box').id;rotation=d.xmat[container].reshape(3,3)
    corners=[]
    for geom in range(m.ngeom):
        if m.geom_bodyid[geom]!=body:continue
        r=d.geom_xmat[geom].reshape(3,3);size=m.geom_size[geom];p=d.geom_xpos[geom]
        if m.geom_type[geom]==mujoco.mjtGeom.mjGEOM_BOX:
            corners.extend(rotation.T@(p+r@(size*np.array([x,y,z]))-d.xpos[container]) for x in [-1,1] for y in [-1,1] for z in [-1,1])
    points=np.array(corners);contained=bool(np.all(points[:,:2]>=[-.192,-.142]) and np.all(points[:,:2]<=[.192,.142]) and points[:,2].min()>=.008 and points[:,2].max()<=.232)
    parcel_geom=m.geom(bag+'_solid_contact').id;penetration=0.
    for name in previous:
        other=m.geom(name+'_solid_contact').id
        penetration=max(penetration,-float(mujoco.mj_geomDistance(m,d,parcel_geom,other,.01,None))*1000)
    result={'bag':bag,'action_id':action_id,'world_xyz_m':d.xpos[body].tolist(),'world_quat_wxyz':d.xquat[body].tolist(),
            'release_world_xyz_m':list(map(float,release_world_xyz)),'trace':trace,'duration_s':3.0,
            'contained_private_score':contained,'final_speed_m_s':float(np.linalg.norm(d.qvel[:3])),
            'maximum_interparcel_penetration_mm':penetration,
            'private_contact_score_passed':bool(contained and penetration<=2 and np.linalg.norm(d.qvel[:3])<.005),
            'scope':'actual gravity/contact settling of rigid parcel against static ellipsoid filler and frozen previously settled parcels; no granular/soft-body dynamics'}
    write_json(state_dir/'private_bin_drops'/(action_id+'.json'),result)
    previous[bag]={k:v for k,v in result.items() if k!='trace'};write_json(state_dir/'private_bin_environment.json',previous)
    # The controller receives no private position or containment result.
    return {'mode':'isolated rigid contact settling','physics_evidence_file':str((state_dir/'private_bin_drops'/(action_id+'.json')).resolve())}


def apply_private_bin_poses(model,data,objects,state_dir,progress=1,drop_bag=None,action_id=None):
    """Renderer/private-scorer API only: do not call from localization or IK."""
    poses=read_json(Path(state_dir)/'private_bin_environment.json',{})
    for name,obj in objects.items():
        if obj.get('destination')!='bin' or name not in poses:continue
        pose=poses[name]
        if name==drop_bag and action_id:
            event=read_json(Path(state_dir)/'private_bin_drops'/(action_id+'.json'),{})
            if event.get('trace'):
                trace=event['trace'];t=float(np.clip(progress,0,1))*event['duration_s'];times=np.array([p['t_s'] for p in trace])
                index=min(max(0,np.searchsorted(times,t,side='right')-1),len(trace)-2)
                a=np.clip((t-times[index])/(times[index+1]-times[index]),0,1)
                pos=(1-a)*np.asarray(trace[index]['world_xyz_m'])+a*np.asarray(trace[index+1]['world_xyz_m'])
                q1=np.asarray(trace[index]['world_quat_wxyz']);q2=np.asarray(trace[index+1]['world_quat_wxyz'])
                if q1@q2<0:q2=-q2
                quat=(1-a)*q1+a*q2;quat/=np.linalg.norm(quat)
                pose={'world_xyz_m':pos,'world_quat_wxyz':quat}
        ident=model.body(name).id;parent=int(model.body_parentid[ident]);r=data.xmat[parent].reshape(3,3)
        model.body_pos[ident]=r.T@(np.asarray(pose['world_xyz_m'])-data.xpos[parent])
        inverse=np.zeros(4);mujoco.mju_negQuat(inverse,data.xquat[parent]);local=np.zeros(4)
        mujoco.mju_mulQuat(local,inverse,np.asarray(pose['world_quat_wxyz']));model.body_quat[ident]=local
    mujoco.mj_forward(model,data)
