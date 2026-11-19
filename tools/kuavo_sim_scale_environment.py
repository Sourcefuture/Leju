#!/usr/bin/env python3
"""Simulator-side scale device. The task algorithm receives RGB OCR only."""
from __future__ import annotations
import os
import copy,json,uuid
import xml.etree.ElementTree as ET
os.environ.setdefault('MUJOCO_GL','egl')
from pathlib import Path
import cv2,mujoco,numpy as np
try:
    from kuavo_planar_task import atomic_json, load_json
    from kuavo_sim_camera import KuavoSimCamera
    from kuavo_scale_vision import make_lcd, read_scale_sequence,texture_theme
    from kuavo_table_vision import FourPointCalibration,TableVision
    from kuavo_rgb_disturbance import perturb_rgb,apply_conditions
except ModuleNotFoundError:
    from tools.kuavo_planar_task import atomic_json, load_json
    from tools.kuavo_sim_camera import KuavoSimCamera
    from tools.kuavo_scale_vision import make_lcd, read_scale_sequence,texture_theme
    from tools.kuavo_table_vision import FourPointCalibration,TableVision
    from tools.kuavo_rgb_disturbance import perturb_rgb,apply_conditions


def weigh_load(scene, bag, mass, load_world_xyz=None, validate_expected=True,load_world_quat=None):
    """Measure a released rigid demo bag on an isolated, equivalent load cell."""
    source=ET.parse(scene).getroot()
    table=copy.deepcopy(source.find(".//body[@name='task_table']"))
    for parent in table.iter('body'):
        for child in list(parent):
            if child.tag=='geom' and child.get('name','').startswith('tag_'):parent.remove(child)
    for child in list(table):
        if child.tag=='body' and child.get('name')!='scale':table.remove(child)
        if child.tag=='site':table.remove(child)
    scale=table.find("body[@name='scale']")
    geom=scale.find("geom[@name='scale_plate']");scale.remove(geom)
    # Preserve the actual plate position/footprint from the scene, rather than
    # silently enlarging it to the full device including its LCD strip.
    display=scale.find("geom[@name='scale_display']")
    display.set('contype','0');display.set('conaffinity','0')
    display.attrib.pop('material',None)
    plate=ET.SubElement(scale,'body',name='load_cell_plate')
    geom.set('mass','0.35');plate.append(geom)
    ET.SubElement(plate,'site',name='load_cell',pos='0 0 0',size='.001')
    root=ET.Element('mujoco',model='isolated planar task scale')
    ET.SubElement(root,'option',timestep='.002',gravity='0 0 -9.81')
    world=ET.SubElement(root,'worldbody');world.append(table)
    ET.SubElement(ET.SubElement(root,'sensor'),'force',name='scale_force',site='load_cell')
    def sample(xml):
        model=mujoco.MjModel.from_xml_string(ET.tostring(xml,encoding='unicode'));data=mujoco.MjData(model)
        forces=[]
        for i in range(1000):
            mujoco.mj_step(model,data)
            if i>=750:forces.append(float(data.sensor('scale_force').data[2]))
        return float(np.mean(forces)),float(np.std(forces))
    tare,_=sample(root)
    transform=np.asarray(load_json(Path(scene).parent/'scene_info.json')['T_world_table'])
    local=np.array([*np.fromstring(scale.get('pos'),sep=' ')[:2],.023])
    xyz=(transform@np.r_[local,1])[:3]
    if load_world_xyz is not None:xyz=np.asarray(load_world_xyz)+[0,0,.003]
    body=copy.deepcopy(source.find(f".//body[@name='{bag}']"))
    body.set('pos',' '.join(map(str,xyz)))
    if load_world_quat is None:
        q=np.zeros(4);mujoco.mju_mulQuat(q,np.fromstring(table.get('quat'),sep=' '),
            np.fromstring(body.get('quat','1 0 0 0'),sep=' '));load_world_quat=q
    body.set('quat',' '.join(map(str,load_world_quat)))
    ET.SubElement(body,'freejoint',name='demo_load_free')
    for g in body.findall('geom'):
        if 'layer' in g.get('name',''):g.set('mass',str(mass/3))
        else:g.set('density','0')
    world.append(body)
    force,std=sample(root)
    measured=abs(force-tare)/9.81
    if not np.isfinite(measured) or std/9.81>.005 or (validate_expected and abs(measured-mass)>.005):
        raise RuntimeError('Load cell failed stability/5 g tolerance check')
    return {'measured_kg':measured,'expected_demo_kg':mass,'std_kg':std/9.81,
            'mode':'isolated contact-physics load cell after kinematic release; no real scale',
            'settle_s':2.0,'tare_force_z_n':tare}



def read_scale_from_environment_rgb(task,bag,noise_level=0,roll_deg=0,rng=None,conditions=None):
    """Mass and simulator positions are private to this device, not returned to task."""
    camera=KuavoSimCamera(task.scene,task.state_dir,1280,720)
    try:
        private_loads=load_json(task.scene.parent/'environment_loads.json',{'bag_white':.2,'bag_green':.5,'bag_black':1.})
        physics=weigh_load(task.scene,bag,float(private_loads[bag]),
                          load_world_xyz=camera.data.body(bag).xpos.copy(),validate_expected=False,
                          load_world_quat=camera.data.body(bag).xquat.copy())
        texture=make_lcd(physics['measured_kg'],texture_theme(camera.model))
        ident=camera.model.texture('scale_lcd_texture').id
        adr=int(camera.model.tex_adr[ident]);size=texture.size
        camera.model.tex_data[adr:adr+size]=texture.ravel()
        camera.renderer._gl_context.make_current()
        mujoco.mjr_uploadTexture(camera.model,camera.renderer._mjr_context,ident)
        ident_cam=camera.model.camera('sim_head').id;angle=np.radians(roll_deg)/2
        original=camera.model.cam_quat[ident_cam].copy()
        mujoco.mju_mulQuat(camera.model.cam_quat[ident_cam],original,np.array([np.cos(angle),0,0,np.sin(angle)]))
        rng=np.random.default_rng(0) if rng is None else rng
        conditions=conditions or {};frames=[];reading=None;views=[]
        viewing_angles=[(0,15),(-5,15),(5,15),(0,12),(0,18),(-8,12),(8,12),(-8,18),(8,18),(0,10),
                        (-5,10),(5,10),(0,8),(-5,8),(5,8),(0,20),(-5,20),(5,20),(-12,15),(12,15)]
        for yaw,pitch in viewing_angles:
            yaw+=float(conditions.get('head_yaw_bias_deg',0));pitch+=float(conditions.get('head_pitch_bias_deg',0))
            if any(not camera.model.joint(name).range[0]<=np.radians(value)<=camera.model.joint(name).range[1] for name,value in [('zhead_1_joint',yaw),('zhead_2_joint',pitch)]):continue
            camera.data.qpos[camera.model.joint('zhead_1_joint').qposadr[0]]=np.radians(yaw)
            camera.data.qpos[camera.model.joint('zhead_2_joint').qposadr[0]]=np.radians(pitch)
            mujoco.mj_forward(camera.model,camera.data)
            rgb=camera.capture_rgb('head');frames.append(apply_conditions(perturb_rgb(rgb,rng,noise_level),rng,conditions));views.append([yaw,pitch])
            if len(frames)>=3:
                try:reading,lcd=read_scale_sequence(frames,minimum=3);break
                except ValueError:continue
        folder=task.state_dir/'scale_images'/uuid.uuid4().hex[:12];folder.mkdir(parents=True,exist_ok=True)
        atomic_json(folder/(bag+'_acquisition.json'),{'head_views_deg':views,'conditions':conditions,'noise_level':noise_level,'optical_roll_deg':roll_deg})
        for index,rgb in enumerate(frames):
            cv2.imwrite(str(folder/(bag+f'_rgb_{index}.png')),cv2.cvtColor(rgb,cv2.COLOR_RGB2BGR))
        cv2.imwrite(str(folder/(bag+'_rgb.png')),cv2.cvtColor(frames[0],cv2.COLOR_RGB2BGR))
        atomic_json(folder/(bag+'_device_diagnostic.json'),physics)
        if reading is None:raise ValueError('No agreeing RGB scale reading after 20 views; leave parcel on scale')
        task.set_head_command(views[-1])
        if reading['measured_kg']<=0:raise ValueError('RGB display does not show a loaded scale')
        cv2.imwrite(str(folder/(bag+'_lcd.png')),cv2.cvtColor(lcd,cv2.COLOR_RGB2BGR))
        reading['rgb_evidence_file']=str((folder/(bag+f'_rgb_{reading["selected_frame_index"]}.png')).resolve())
        reading['lcd_evidence_file']=str((folder/(bag+'_lcd.png')).resolve())
        # Simulator measurements are kept in the device diagnostic file only.
        return reading
    finally:camera.close()
