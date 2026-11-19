#!/usr/bin/env python3
"""Configure an isolated Kuavo-5 acceptance scene; never execute real hardware."""
from __future__ import annotations
import argparse, json, math, shutil, xml.etree.ElementTree as ET
from pathlib import Path
import mujoco
import numpy as np

ROOT = Path(__file__).resolve().parents[1]
DEFAULT_SOURCE = ROOT / 'outputs/kuavo_sim/planar_rgb_weighing/scene.xml'


def numbers(values):
    return ' '.join(f'{float(v):.12g}' for v in values)


def yaw_quat(deg):
    angle = math.radians(float(deg))/2
    return np.array([math.cos(angle),0,0,math.sin(angle)])


def build(source, output, config):
    source, output = Path(source).resolve(), Path(output).resolve()
    if source.parent == output:
        raise ValueError('Use a new output directory; preserve the source scene')
    output.mkdir(parents=True, exist_ok=True)
    tree = ET.parse(source); xml = tree.getroot()
    # Rebase assets before moving the XML. Copy mutable LCD/marker images so
    # independent scenarios cannot accidentally share an edited texture.
    for texture in xml.findall('asset/texture'):
        if texture.get('file'):
            asset = Path(texture.get('file'))
            if not asset.is_absolute(): asset = source.parent/asset
            target = output/asset.name
            shutil.copyfile(asset, target)
            texture.set('file', str(target))
    if 'lcd_theme' in config:
        from kuavo_scale_vision import make_lcd
        import cv2
        lcd=make_lcd(0,config['lcd_theme']);path=output/'scale_lcd.png'
        cv2.imwrite(str(path),cv2.cvtColor(lcd,cv2.COLOR_RGB2BGR))
        xml.find("asset/texture[@name='scale_lcd_texture']").set('file',str(path))
    for asset in xml.findall('asset/mesh'):
        if asset.get('file') and not Path(asset.get('file')).is_absolute():
            base = Path(xml.find('compiler').get('meshdir',str(source.parent)))
            if not base.is_absolute(): base = source.parent/base
            asset.set('file',str((base/asset.get('file')).resolve()))
    table = xml.find("worldbody/body[@name='task_table']")
    pos = np.fromstring(table.get('pos'),sep=' ')
    old_pos = pos.copy()
    pos += np.array(config.get('table_translation_m',[0,0,0]),float)
    if 'table_near_edge_x_m' in config: pos[0] = float(config['table_near_edge_x_m'])
    quat = np.fromstring(table.get('quat'),sep=' '); rotated=np.zeros(4)
    mujoco.mju_mulQuat(rotated,yaw_quat(config.get('table_yaw_deg',0)),quat)
    if config.get('table_yaw_pivot','near_left_corner')=='center':
        before=np.zeros(9);after=np.zeros(9);mujoco.mju_quat2Mat(before,quat);mujoco.mju_quat2Mat(after,rotated)
        pos+=(before.reshape(3,3)-after.reshape(3,3))@np.array([.6,.3,0])
    table.set('pos',numbers(pos));table.set('quat',numbers(rotated))
    for leg in table.findall('geom'):
        if leg.get('name','').startswith('white_leg_'):
            center=np.fromstring(leg.get('pos'),sep=' ');size=np.fromstring(leg.get('size'),sep=' ')
            center[2]=-(pos[2]+.031)/2;size[2]=(pos[2]-.031)/2
            if size[2]<=0:raise ValueError('Table height is invalid')
            leg.set('pos',numbers(center));leg.set('size',numbers(size))
    if config.get('hide_calibration_sites',True):
        for site in table.findall('site'):
            site.set('rgba','0 0 0 0')
    for name, spec in config.get('items',{}).items():
        body=table.find(f"body[@name='{name}']")
        if body is None: raise ValueError(f'Unknown item: {name}')
        p=np.fromstring(body.get('pos'),sep=' ')
        if 'xy_m' in spec: p[:2]=spec['xy_m']
        body.set('pos',numbers(p));body.set('quat',numbers(yaw_quat(spec.get('yaw_deg',0))))
    # Same 140 cm cloth crosswise: an explicitly folded/repositioned drape,
    # not an invisible removal of the collision warning.
    near_drop=float(config.get('near_cloth_drop_m',.4))
    if not .02<=near_drop<=.76: raise ValueError('near_cloth_drop_m must be .02..76 m')
    far_drop=.8-near_drop
    for side,drop in [('near',near_drop),('far',far_drop)]:
        g=table.find(f"geom[@name='cloth_{side}']")
        p=np.fromstring(g.get('pos'),sep=' ');s=np.fromstring(g.get('size'),sep=' ')
        p[2]=-drop/2;s[2]=drop/2;g.set('pos',numbers(p));g.set('size',numbers(s))
    if config.get('table_marker_centers_m'):
        for ident,xy in config['table_marker_centers_m'].items():
            g=table.find(f"geom[@name='tag_{int(ident)}']")
            if g is None: raise ValueError(f'Table marker {ident} absent from source scene')
            g.set('pos',numbers([*xy,.0005]))
        (output/'marker_layout.json').write_text(json.dumps({'table_centers_m':config['table_marker_centers_m']},indent=2)+'\n')
    # Apply robot placement as a fixed simulation reset, including foot pose.
    model=mujoco.MjModel.from_xml_path(str(source));data=mujoco.MjData(model)
    mujoco.mj_resetDataKeyframe(model,data,0)
    data.qpos[:3]+=np.array(config.get('robot_translation_m',[0,0,0]),float)
    q=np.zeros(4);mujoco.mju_mulQuat(q,yaw_quat(config.get('robot_yaw_deg',0)),data.qpos[3:7])
    data.qpos[3:7]=q
    for j,value in [('zhead_1_joint',config.get('head_yaw_deg',0)),('zhead_2_joint',config.get('head_pitch_deg',10))]:
        ident=model.joint(j).id;angle=math.radians(float(value))
        if not model.jnt_range[ident,0]<=angle<=model.jnt_range[ident,1]: raise ValueError(f'{j} exceeds model range')
        data.qpos[model.jnt_qposadr[ident]]=angle
    xml.find('keyframe/key').set('qpos',numbers(data.qpos))
    head=xml.find(".//camera[@name='sim_head']")
    if 'camera_mount_translation_m' in config:
        head.set('pos',numbers(np.fromstring(head.get('pos'),sep=' ')+config['camera_mount_translation_m']))
    if 'camera_mount_roll_deg' in config:
        original=np.fromstring(head.get('quat'),sep=' ') if head.get('quat') else model.cam_quat[model.camera('sim_head').id]
        result=np.zeros(4);mujoco.mju_mulQuat(result,original,yaw_quat(config['camera_mount_roll_deg']))
        head.attrib.pop('xyaxes',None);head.set('quat',numbers(result))
    if 'camera_fovy_deg' in config:
        fovy=float(config['camera_fovy_deg'])
        if not 30<=fovy<=100: raise ValueError('Camera fovy must be 30..100 degrees')
        head.set('fovy',str(fovy))
    for camera in xml.findall('worldbody/camera'):
        p=np.fromstring(camera.get('pos'),sep=' ');p+=pos-old_pos;camera.set('pos',numbers(p))
    scene=output/'scene.xml';tree.write(scene,encoding='utf-8',xml_declaration=True)
    m=mujoco.MjModel.from_xml_path(str(scene));d=mujoco.MjData(m);mujoco.mj_resetDataKeyframe(m,d,0);mujoco.mj_forward(m,d)
    transform=np.eye(4);tableid=m.body('task_table').id
    transform[:3,:3]=d.xmat[tableid].reshape(3,3);transform[:3,3]=d.xpos[tableid]
    info=json.loads((source.parent/'scene_info.json').read_text())
    info['stage']='configurable acceptance environment; robot placement is a simulation reset'
    info['table']['height_m']=float(pos[2])
    near_mid=(d.site('calib_A_near_left').xpos+d.site('calib_B_near_right').xpos)/2
    info['T_world_table']=transform.tolist();info['table']['robot_to_near_edge_m']=float(near_mid[0])
    info['table']['robot_to_near_edge_definition']='world X of near-edge midpoint, not torso or foot clearance; consult placement_report.json'
    for name,entry in info['calibration_points'].items():
        entry['world_xyz_m']=(transform@np.r_[entry['table_xyz_m'],1])[:3].tolist()
    for name,entry in info['items'].items():
        body=m.body(name).id;entry['xy']=m.body_pos[body,:2].tolist()
        entry['yaw_deg']=config.get('items',{}).get(name,{}).get('yaw_deg',0)
    (output/'scene_info.json').write_text(json.dumps(info,indent=2)+'\n')
    corners={key:d.site('calib_'+name).xpos.tolist() for key,name in zip('ABCD',['A_near_left','B_near_right','C_far_right','D_far_left'])}
    (output/'assumed_manual_corners.json').write_text(json.dumps({'reference_frame':'mujoco_world','clearance_m':0,
        'corners_robot':corners,'provenance':'environment-generated assumed known coordinates; NOT operator-confirmed arm arrivals'},indent=2)+'\n')
    for name in ['environment_loads.json','crouch_assumptions.json']:
        if (source.parent/name).is_file(): shutil.copyfile(source.parent/name,output/name)
    if 'loads_kg' in config: (output/'environment_loads.json').write_text(json.dumps(config['loads_kg'],indent=2)+'\n')
    if 'view_conditions' in config:
        (output/'view_conditions.json').write_text(json.dumps(config['view_conditions'],indent=2)+'\n')
    report={'robot':'Kuavo-5.2 biped_s52','root_world_xyz_m':d.qpos[:3].tolist(),
        'corner_A_world_xyz_m':d.site('calib_A_near_left').xpos.tolist(),
        'near_edge_midpoint_world_xyz_m':near_mid.tolist(),
        'feet_world_xyz_m':{s:d.body('leg_'+s+'6_link').xpos.tolist() for s in ['l','r']},
        'near_edge_minus_pelvis_world_x_m':float(pos[0]-d.qpos[0]),
        'near_edge_midpoint_minus_pelvis_world_x_m':float(near_mid[0]-d.qpos[0]),
        'near_cloth_drop_m':near_drop,'far_cloth_drop_m':far_drop,
        'scope':'geometric placement report; no physical stance transition commanded'}
    feet_center=np.mean([d.body('leg_'+s+'6_link').xpos for s in ['l','r']],axis=0)
    edge_normal=transform[:3,1]
    report['feet_center_to_near_edge_perpendicular_m']=float(edge_normal@(near_mid-feet_center))
    report['pelvis_to_near_edge_perpendicular_m']=float(edge_normal@(near_mid-d.qpos[:3]))
    (output/'placement_report.json').write_text(json.dumps(report,indent=2)+'\n')
    (output/'scenario.json').write_text(json.dumps(config,indent=2)+'\n')
    return scene


def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--source-scene',type=Path,default=DEFAULT_SOURCE)
    p.add_argument('--config',type=Path,required=True);p.add_argument('--output',type=Path,required=True)
    a=p.parse_args();print(build(a.source_scene,a.output,json.loads(a.config.read_text())))


if __name__=='__main__': main()
