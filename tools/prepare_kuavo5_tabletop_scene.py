#!/usr/bin/env python3
"""Build the dimensioned competition table for Kuavo-5.2 scene review."""

from __future__ import annotations

import argparse
import json
import os
from pathlib import Path
import xml.etree.ElementTree as ET

os.environ.setdefault("MUJOCO_GL", "egl")
os.environ.setdefault("XDG_CACHE_HOME", "/tmp/kuavo-mujoco-cache")

import cv2
import mujoco
import numpy as np

try:
    from prepare_kuavo5_weighing_scene import MODEL_DIR, box, camera, numbers, save
except ModuleNotFoundError:
    from tools.prepare_kuavo5_weighing_scene import MODEL_DIR, box, camera, numbers, save

ROOT = Path(__file__).resolve().parents[1]
OUTPUT = ROOT / "outputs/kuavo_sim/tabletop_calibration"
POINTS = {
    "A_near_left": [0,0,0], "B_near_right": [1.2,0,0],
    "C_far_right": [1.2,.6,0], "D_far_left": [0,.6,0], "E_center": [.6,.3,0],
}
LAYOUT = {
    "material_box": {"xy": [.97,.30], "size_m": [.4,.3,.23]},
    "bag_white": {"xy": [.16,.10], "size_m": [.30,.20,.012]},
    "bag_green": {"xy": [.16,.30], "size_m": [.30,.20,.012]},
    "bag_black": {"xy": [.16,.50], "size_m": [.30,.20,.012]},
    "scale": {"xy": [.60,.30], "size_m": [.23,.17,.02]},
}


def build(output=OUTPUT, table_height=.76, table_gap=.22, markers=False):
    if not .45 <= table_height <= 1.2 or not .05 <= table_gap <= 1.0:
        raise ValueError("table-height must be 0.45..1.2 m; table-gap must be 0.05..1.0 m")
    output = Path(output).resolve()
    output.mkdir(parents=True, exist_ok=True)
    scene = ET.parse(MODEL_DIR / "xml/biped_s52.xml").getroot()
    scene.set("model", "Kuavo-5.2 competition tabletop")
    scene.find("compiler").set("meshdir", str((MODEL_DIR / "meshes").resolve()))
    ET.SubElement(scene, "option", timestep="0.002", gravity="0 0 -9.81")
    visual = ET.SubElement(scene, "visual")
    ET.SubElement(visual, "global", offwidth="1280", offheight="960")
    ET.SubElement(visual, "headlight", ambient="0.25 0.25 0.25", diffuse="0.25 0.25 0.25")
    world = scene.find("worldbody")
    ET.SubElement(world, "light", pos="0.5 -0.8 3", dir="0 0 -1", directional="true",
                  diffuse="0.5 0.5 0.5")
    ET.SubElement(world, "geom", name="tabletop_floor", type="plane", size="3 3 .05",
                  rgba="0.18 0.22 0.27 1", contype="8", conaffinity="8")
    # Table X goes to robot right (-world Y); table Y goes forward (+world X).
    origin = [table_gap,.6,table_height]
    table = ET.SubElement(world, "body", name="task_table", pos=numbers(origin),
                          quat="0.707106781187 0 0 -0.707106781187")
    box(table, "table_top", [.6,.3,-.016], [.6,.3,.015], "0.63 0.44 0.27 1")
    for i,(x,y) in enumerate(((.055,.055),(1.145,.055),(.055,.545),(1.145,.545))):
        box(table,f"white_leg_{i}",[x,y,-(table_height+.031)/2],
            [.025,.025,(table_height-.031)/2],"0.93 0.93 0.90 1")
    cloth_color = "0.80 0.72 0.57 1"
    box(table,"cloth_top",[.6,.3,-.0005],[.6,.3,.0005],cloth_color)
    # Approximate 1.80 x 1.40 m cloth: 0.30 m end and 0.40 m side drops.
    for name,pos,size in (
        ("left",[0,.3,-.15],[.001,.3,.15]),
        ("right",[1.2,.3,-.15],[.001,.3,.15]),
        ("near",[.6,0,-.20],[.6,.001,.20]),
        ("far",[.6,.6,-.20],[.6,.001,.20]),
    ):
        box(table,f"cloth_{name}",pos,size,cloth_color)
    for name,xyz in POINTS.items():
        ET.SubElement(table,"site",name=f"calib_{name}",pos=numbers(xyz),
                      type="sphere",size="0.009",rgba="1 0.1 0.1 1")
    # Outer dimensions include the bottom and wall thickness.
    spec=LAYOUT["material_box"]
    x,y=spec["xy"]
    bin_body=ET.SubElement(table,"body",name="material_box",pos=numbers([x,y,0]))
    box(bin_body,"material_box_floor",[0,0,.005],[.20,.15,.005],"0.90 0.91 0.88 1")
    for name,pos,size in (
        ("left",[-.195,0,.12],[.005,.15,.11]),
        ("right",[.195,0,.12],[.005,.15,.11]),
        ("near",[0,-.145,.12],[.19,.005,.11]),
        ("far",[0,.145,.12],[.19,.005,.11]),
    ):
        box(bin_body,f"material_box_{name}",pos,size,"0.87 0.89 0.85 1")
    # Twelve visual representatives, not a rigid/deformable particle simulation.
    rng=np.random.default_rng(12)
    for i in range(12):
        px=-.135+(i%4)*.09
        py=-.08+(i//4)*.08
        radius=[.01,.02,.03,.04,.05][i%5]
        ET.SubElement(bin_body,"geom",name=f"filler_{i}",type="ellipsoid",
                      pos=numbers([px,py,.065+rng.uniform(0,.015)]),
                      size=numbers([radius,.7*radius,.6*radius]),
                      rgba="0.94 0.87 0.70 1",contype="0",conaffinity="0")
    bag_colors={"white":"0.97 0.97 0.95 1", "green":"0.13 0.48 0.24 1", "black":"0.045 0.045 0.05 1"}
    for color,rgba in bag_colors.items():
        x,y=LAYOUT[f"bag_{color}"]["xy"]
        body=ET.SubElement(table,"body",name=f"bag_{color}",pos=numbers([x,y,0]))
        for layer in range(3):
            box(body,f"bag_{color}_layer_{layer}",[0,0,.002+layer*.004],
                [.15,.1,.002],rgba)
        box(body,f"bag_{color}_seam",[.137,0,.012],[.003,.095,.0002],"0.50 0.50 0.48 1")
    x,y=LAYOUT["scale"]["xy"]
    scale=ET.SubElement(table,"body",name="scale",pos=numbers([x,y,0]))
    box(scale,"scale_base",[0,0,.005],[.115,.085,.005],"0.16 0.17 0.19 1")
    box(scale,"scale_plate",[0,.019,.015],[.115,.066,.005],"0.70 0.73 0.75 1")
    try:
        from kuavo_scale_vision import make_lcd
    except ModuleNotFoundError:
        from tools.kuavo_scale_vision import make_lcd
    lcd=make_lcd(0);lcd_path=output/'scale_lcd.png'
    cv2.imwrite(str(lcd_path),cv2.cvtColor(lcd,cv2.COLOR_RGB2BGR))
    asset=scene.find('asset')
    ET.SubElement(asset,'texture',name='scale_lcd_texture',type='2d',file=str(lcd_path.resolve()))
    ET.SubElement(asset,'material',name='scale_lcd_material',texture='scale_lcd_texture',texrepeat='1 1')
    ET.SubElement(scale,'geom',name='scale_display',type='box',pos='0 -.0675 .0195',
                  size='.085 .0165 .0005',material='scale_lcd_material',contype='0',conaffinity='0')
    if markers:
        add_visual_markers(scene, output, table)
    camera(world,"weighing_overview",[2.1,-2.5,2.05],[table_gap+.3,0,.95])
    ET.SubElement(world,"camera",name="weighing_top",pos=numbers([table_gap+.3,0,2.2]),
                  xyaxes="0 -1 0 1 0 0",fovy="48")
    ET.SubElement(scene.find(".//body[@name='head_camera_depth_link']"),"camera",
                  name="sim_head",pos="0.015 0 0",xyaxes="0 -1 0 0.642787610 0 0.766044443",fovy="60")
    for side in ('l','r'):
        ET.SubElement(scene.find(f".//body[@name='zarm_{side}7_link']"),"camera",
                      name=f"sim_wrist_{side}",pos="0.025 0.065 -0.045",xyaxes="1 0 0 0 1 0",fovy="60")
    path=output/'scene.xml'
    save(scene,path)
    model=mujoco.MjModel.from_xml_path(str(path))
    home=model.qpos0.copy()
    base=model.body('base_link').id
    home[model.jnt_qposadr[model.body_jntadr[base]]+2]=.98
    ET.SubElement(ET.SubElement(scene,'keyframe'),'key',name='home',qpos=numbers(home))
    save(scene,path)
    rotation=np.array([[0,1,0],[-1,0,0],[0,0,1]],float)
    transform=np.eye(4)
    transform[:3,:3]=rotation
    transform[:3,3]=origin
    info={
        'robot':'Kuavo-5.2 biped_s52, same official model as prior simulations',
        'stage':'basic scene for user review; no calibration solving or grasp execution',
        'table':{'size_m':[1.2,.6],'height_m':table_height,'robot_to_near_edge_m':table_gap,
                 'origin':'near-left corner on tabletop','axes':'X right, Y away from robot, Z up'},
        'T_world_table':transform.tolist(),
        'calibration_points':{name:{'table_xyz_m':xyz,'world_xyz_m':(rotation@xyz+origin).tolist()}
                              for name,xyz in POINTS.items()},
        'items':{name:{**spec,'xy':([.25,spec['xy'][1]] if markers and name.startswith('bag_') else spec['xy'])}
                 for name,spec in LAYOUT.items()},'cloth':{'flat_size_m':[1.8,1.4],'representation':'top and four static draped panels'},
        'visual_markers': bool(markers),
        'assumptions':['table height 0.76 m and robot gap 0.22 m are adjustable defaults',
                       'bag bundle thickness 12 mm; 3 layers for display, actual bundle count unknown',
                       'filler 2..10 cm represented by 12 static particles; supplied total 5 jin (2.5 kg)',
                       'material box color is illustrative; wall thickness assumed 10 mm',
                       'scale 5 kg capacity and 0.1 g display resolution are specifications, not validated accuracy',
                       'robot reachability, collisions, soft-body behavior and automatic weighing are unvalidated'],
    }
    (output/'scene_info.json').write_text(json.dumps(info,ensure_ascii=False,indent=2)+'\n')
    return path


def add_visual_markers(scene, output, table):
    """Optional RGB-only labels; marker placement is a physical setup convention."""
    dictionary=cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
    asset=scene.find('asset')
    definitions=[(0,table,[.05,.05,.0005]),(1,table,[1.15,.05,.0005]),
                 (2,table,[1.15,.55,.0005]),(3,table,[.05,.55,.0005]),
                 (4,table,[.60,.05,.0005]),(5,table,[.60,.55,.0005]),
                 (10,table.find("body[@name='scale']"),[-.06,-.14,.0005]),
                 (12,table.find("body[@name='scale']"),[.06,-.14,.0005]),
                 (11,table.find("body[@name='material_box']"),[-.09,-.25,.0005]),
                 (13,table.find("body[@name='material_box']"),[.09,-.25,.0005])]
    for ident,parent,pos in definitions:
        # Quiet white margin: 60 mm black tag in a 75 mm paper patch.
        marker=cv2.aruco.generateImageMarker(dictionary,ident,160)
        patch=np.pad(marker,20,constant_values=255)
        path=Path(output)/f'marker_{ident}.png';cv2.imwrite(str(path),patch)
        ET.SubElement(asset,'texture',name=f'tag_tex_{ident}',type='2d',file=str(path.resolve()))
        ET.SubElement(asset,'material',name=f'tag_mat_{ident}',texture=f'tag_tex_{ident}',
                      texrepeat='1 1',reflectance='0',rgba='1 1 1 1')
        ET.SubElement(parent,'geom',name=f'tag_{ident}',type='box',pos=numbers(pos),
                      size='.0375 .0375 .0002',material=f'tag_mat_{ident}',contype='0',conaffinity='0',density='0')
    # Leave the tags visible beside the left-hand bag stacks.
    for name in ('bag_white','bag_green','bag_black'):
        body=table.find(f"body[@name='{name}']")
        p=np.fromstring(body.get('pos'),sep=' ');p[0]=.25;body.set('pos',numbers(p))


def preview(output):
    from kuavo_sim_camera import KuavoSimCamera
    output=Path(output).resolve()
    reader=KuavoSimCamera(output/'scene.xml',ROOT/'outputs/robot_control/kuavo5_tabletop',960,540)
    info=json.loads((output/'scene_info.json').read_text())
    try:
        for name in ('overview','top','head'):
            rgb,_,metadata=reader.capture(name)
            bgr=cv2.cvtColor(rgb,cv2.COLOR_RGB2BGR)
            transform=np.asarray(metadata['T_world_camera_optical'])
            k=np.asarray(metadata['K'])
            for point_name,point in info['calibration_points'].items():
                p=transform[:3,:3].T@(np.asarray(point['world_xyz_m'])-transform[:3,3])
                if p[2]<=0:
                    continue
                uv=k@p; u,v=(uv[:2]/uv[2]).round().astype(int)
                if 0<=u<960 and 0<=v<540:
                    cv2.putText(bgr,point_name[0],(int(u)+9,int(v)-9),cv2.FONT_HERSHEY_SIMPLEX,.7,(0,0,255),2)
            cv2.rectangle(bgr,(0,0),(960,62),(28,28,28),-1)
            cv2.putText(bgr,'KUAVO 5.2 | 120 x 60 cm competition table',(20,26),cv2.FONT_HERSHEY_SIMPLEX,.65,(255,255,255),2)
            cv2.putText(bgr,f"height {info['table']['height_m']:.2f} m (assumed) | A-D corners | E center",
                        (20,51),cv2.FONT_HERSHEY_SIMPLEX,.5,(230,230,230),1)
            cv2.imwrite(str(output/f'{name}.png'),bgr)
            print(output/f'{name}.png')
    finally:
        reader.close()
    # Dimensioned plan: near edge at bottom, table +X right and +Y upward.
    image=np.full((760,1360,3),245,np.uint8)
    def pixel(x,y): return (round(80+1000*x),round(680-1000*y))
    cv2.rectangle(image,pixel(0,.6),pixel(1.2,0),(156,185,204),-1)
    palette={'material_box':(200,210,210),'bag_white':(255,255,255),
             'bag_green':(70,150,60),'bag_black':(45,45,45),'scale':(180,180,180)}
    for name,spec in LAYOUT.items():
        x,y=spec['xy'];w,d,_=spec['size_m']
        cv2.rectangle(image,pixel(x-w/2,y+d/2),pixel(x+w/2,y-d/2),palette[name],-1)
        cv2.rectangle(image,pixel(x-w/2,y+d/2),pixel(x+w/2,y-d/2),(50,50,50),2)
        cv2.putText(image,name,pixel(x-w/2+.01,y),cv2.FONT_HERSHEY_SIMPLEX,.5,
                    (250,250,250) if name=='bag_black' else (20,20,20),1)
    for name,(x,y,z) in POINTS.items():
        u,v=pixel(x,y);cv2.circle(image,(u,v),9,(0,0,240),-1)
        cv2.putText(image,name[0],(u+12,v-12),cv2.FONT_HERSHEY_SIMPLEX,.8,(0,0,220),2)
    cv2.putText(image,'1200 mm: X right | 600 mm: Y away | robot at bottom',(80,40),cv2.FONT_HERSHEY_SIMPLEX,.75,(20,20,20),2)
    cv2.putText(image,'Near edge / robot side',(500,735),cv2.FONT_HERSHEY_SIMPLEX,.7,(20,20,20),2)
    cv2.imwrite(str(output/'layout.png'),image)
    print(output/'layout.png')


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output',type=Path,default=OUTPUT)
    parser.add_argument('--table-height',type=float,default=.76)
    parser.add_argument('--table-gap',type=float,default=.22)
    parser.add_argument('--render',action='store_true')
    parser.add_argument('--preview-only',action='store_true')
    parser.add_argument('--markers',action='store_true',help='add optional RGB fiducials for vision robustness experiments')
    args=parser.parse_args()
    if not args.preview_only:
        print(build(args.output,args.table_height,args.table_gap,args.markers))
    if args.render or args.preview_only:
        preview(args.output)


if __name__=='__main__':
    main()
