#!/usr/bin/env python3
"""Replay recorded RGB-driven task actions and audit every rendered hand pose."""
from __future__ import annotations
import argparse,json,os,hashlib
from pathlib import Path
os.environ.setdefault('MUJOCO_GL','egl')
import cv2,mujoco,numpy as np
from kuavo_sim_bin_environment import apply_private_bin_poses
from kuavo_video_codec import finalize as finalize_video


def caption_strip(text,font_path=None):
    """Cache a readable UTF-8 caption; sensor images are never changed."""
    from PIL import Image,ImageDraw,ImageFont
    fonts=([Path(font_path)] if font_path else [])+[
        Path('/mnt/c/Windows/Fonts/msyh.ttc'),
        Path('/usr/share/fonts/opentype/noto/NotoSansCJK-Regular.ttc')]
    chosen=next((p for p in fonts if p.is_file()),None)
    if chosen is None:return None,None
    canvas=Image.new('RGB',(1280,32),(24,24,24));font=ImageFont.truetype(str(chosen),17)
    ImageDraw.Draw(canvas).text((12,2),text,font=font,fill=(215,230,240))
    return cv2.cvtColor(np.asarray(canvas),cv2.COLOR_RGB2BGR),str(chosen)
try:
    from kuavo_planar_task import sample_motion,apply_objects,load_json
    from kuavo_mujoco_live_viewer import arm_addresses
    from kuavo_scale_vision import write_lcd_texture
    from kuavo_rgb_disturbance import perturb_rgb,apply_conditions
except ModuleNotFoundError:
    from tools.kuavo_planar_task import sample_motion,apply_objects,load_json
    from tools.kuavo_mujoco_live_viewer import arm_addresses
    from tools.kuavo_scale_vision import write_lcd_texture
    from tools.kuavo_rgb_disturbance import perturb_rgb,apply_conditions


def run(args):
    model=mujoco.MjModel.from_xml_path(str(args.scene.resolve()));data=mujoco.MjData(model)
    if model.nkey:mujoco.mj_resetDataKeyframe(model,data,0)
    addresses=arm_addresses(model)
    records=[json.loads(line) for line in (args.state_dir/'history.jsonl').read_text().splitlines()]
    records=[r for r in records if r.get('kind')=='planar_task' and r.get('status')=='executed']
    if not records:raise ValueError('No executed planar actions to replay')
    data.qpos[addresses]=records[0]['trajectory_arm14_rad'][0]
    mujoco.mj_forward(model,data)
    args.output.parent.mkdir(parents=True,exist_ok=True)
    size=(1280,720) if args.sensor_panel else (960,540)
    writer=cv2.VideoWriter(str(args.output),cv2.VideoWriter_fourcc(*'mp4v'),args.fps,size)
    if not writer.isOpened():raise RuntimeError('Video writer failed')
    angles=[];frames=0;last_weight=None;rng=np.random.default_rng(args.seed)
    conditions=load_json(args.view_conditions,{}) if args.view_conditions else {}
    caption,font=caption_strip(f'{args.playback_speed:g}x simulation time | '+(args.label or args.scene.parent.name),args.caption_font)
    if args.camera_roll_deg:
        ident=model.camera('sim_head').id;q=model.cam_quat[ident].copy();r=np.radians(args.camera_roll_deg)/2
        mujoco.mju_mulQuat(model.cam_quat[ident],q,np.array([np.cos(r),0,0,np.sin(r)]))
    evidence_cache={}
    audit=load_json(args.scene.parent/'acceptance_audit.json',{})
    audit_notice=' | INDEPENDENT AUDIT FAILED' if audit.get('passed') is False else ''
    try:
        with mujoco.Renderer(model,height=540,width=960) as renderer:
            for index,record in enumerate(records):
                for name,v in zip(['zhead_1_joint','zhead_2_joint'],record.get('head_yaw_pitch_deg',[])):
                    data.qpos[model.joint(name).qposadr[0]]=np.radians(v)
                start=data.qpos[addresses].copy()
                value=float(record.get('scale_display_kg',0))
                if value!=last_weight:
                    ident=write_lcd_texture(model,value)
                    if ident is not None:
                        renderer._gl_context.make_current();mujoco.mjr_uploadTexture(model,renderer._mjr_context,ident)
                    last_weight=value
                count=max(2,round(record.get('actual_duration_s',2)*args.fps/args.playback_speed))
                for j in range(count):
                    fraction=(j+1)/count
                    data.qpos[addresses]=sample_motion(record,start,fraction)
                    mujoco.mj_forward(model,data)
                    apply_objects(model,data,record.get('task_objects',{}),fraction,
                                  record.get('bag') if record.get('event')=='drop' else None)
                    apply_private_bin_poses(model,data,record.get('task_objects',{}),args.state_dir,fraction,
                                  record.get('bag') if record.get('event')=='drop' else None,record.get('action_id'))
                    for side,normal in [('l',[0,-1,0]),('r',[0,1,0])]:
                        direction=data.body('zarm_'+side+'7_link').xmat.reshape(3,3)@normal
                        angle=np.degrees(np.arccos(np.clip(direction@np.array([0,0,-1]),-1,1)))
                        angles.append(float(angle))
                    renderer.update_scene(data,camera=args.camera)
                    # Overview is an illustrative CAD view, not sensor evidence.
                    renderer.scene.flags[mujoco.mjtRndFlag.mjRND_SHADOW]=False
                    renderer.scene.flags[mujoco.mjtRndFlag.mjRND_REFLECTION]=False
                    bgr=cv2.cvtColor(renderer.render(),cv2.COLOR_RGB2BGR)
                    if args.sensor_panel:
                        world=bgr
                        renderer.update_scene(data,camera='sim_head')
                        renderer.scene.flags[mujoco.mjtRndFlag.mjRND_SHADOW]=True
                        renderer.scene.flags[mujoco.mjtRndFlag.mjRND_REFLECTION]=True
                        head=apply_conditions(perturb_rgb(renderer.render().copy(),rng,args.noise_level),rng,conditions)
                        pane=np.full((720,1280,3),24,np.uint8)
                        pane[100:550,:800]=cv2.resize(world,(800,450))
                        pane[100:370,800:]=cv2.resize(cv2.cvtColor(head,cv2.COLOR_RGB2BGR),(480,270))
                        evidence_file=record.get('weighing',{}).get('rgb_evidence_file')
                        if not evidence_file and record.get('vision_observation_file'):
                            key=record['vision_observation_file']
                            if key not in evidence_cache:evidence_cache[key]=load_json(key,{})
                            evidence_file=evidence_cache[key].get('reference_rgb_file')
                        if evidence_file:
                            evidence=cv2.imread(str(evidence_file))
                            if evidence is not None:pane[420:690,800:]=cv2.resize(evidence,(480,270))
                        for y,text in [(28,'KUAVO 5.2 | KINEMATIC ASSISTED GRASP | RGB + current-state IK'+audit_notice),
                                       (60,f'{args.playback_speed:g}x simulation time | '+(args.label or args.scene.parent.name)),
                                       (88,f'{index+1}/{len(records)} {record["name"]} | LCD {value*1000:.1f} g'),
                                       (583,'Left: CAD overview (shadows off). Top-right: head RGB, original lighting.'),
                                       (615,'Bottom-right: archived RGB actually used for this route / scale read, before motion.'),
                                       (650,'A successful assisted replay does NOT prove articulated-finger grasp or real robot execution.')]:
                            cv2.putText(pane,text,(12,y),cv2.FONT_HERSHEY_SIMPLEX,.52,(215,230,240),1)
                        if caption is not None:pane[42:74]=caption
                        cv2.putText(pane,'Simulated head RGB',(810,392),cv2.FONT_HERSHEY_SIMPLEX,.50,(110,230,160),1)
                        cv2.putText(pane,'Actual input RGB (acquisition snapshot)',(810,413),cv2.FONT_HERSHEY_SIMPLEX,.45,(110,230,160),1)
                        bgr=pane
                    else:
                        cv2.rectangle(bgr,(0,0),(960,72),(24,24,24),-1)
                        cv2.putText(bgr,'KUAVO 5.2 | RGB fiducials + four corner XYZ | assisted grasp',(16,26),cv2.FONT_HERSHEY_SIMPLEX,.62,(255,255,255),2)
                        cv2.putText(bgr,f'{index+1}/{len(records)} {record["name"]} | LCD {value*1000:.1f} g',(16,54),cv2.FONT_HERSHEY_SIMPLEX,.58,(100,235,200),1)
                    writer.write(bgr);frames+=1
                if index%10==0:print(f'render {index+1}/{len(records)}',flush=True)
                if record.get('event')=='weigh':cv2.imwrite(str(args.output.with_name(record['bag']+'_weigh.png')),bgr)
            cv2.imwrite(str(args.output.with_suffix('.png')),bgr)
    finally:writer.release()
    codec=finalize_video(args.output,frames)
    report={'scene':str(args.scene.resolve()),'state_dir':str(args.state_dir.resolve()),
            'history_sha256':hashlib.sha256((args.state_dir/'history.jsonl').read_bytes()).hexdigest(),
            'frames':frames,'fps':args.fps,'duration_s':frames/args.fps,
            'actions':len(records),'max_palm_normal_error_deg':max(angles),
            'playback_speed':args.playback_speed,'overview_shadows':False,'head_original_lighting':True,
            'caption_font':font,
            'independent_audit_passed':audit.get('passed'),
            'sensor_panel':args.sensor_panel,'label':args.label,'noise_level':args.noise_level,'view_conditions':conditions,
            'palm_down_within_1deg':max(angles)<=1,
            'physics_scope':'recorded kinematic arms and assisted attachment/drop; not MPC or finger contact physics'}
    report.update(codec)
    args.output.with_suffix('.json').write_text(json.dumps(report,indent=2)+'\n')
    print(json.dumps(report,indent=2),flush=True)


def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--scene',type=Path,required=True);p.add_argument('--state-dir',type=Path,required=True)
    p.add_argument('--output',type=Path,required=True);p.add_argument('--fps',type=int,default=10)
    p.add_argument('--playback-speed',type=float,default=1,help='Declared video speed; every recorded stage remains in the replay')
    p.add_argument('--caption-font',type=Path,help='Optional UTF-8 font for the condition label')
    p.add_argument('--camera',default='weighing_overview')
    p.add_argument('--sensor-panel',action='store_true');p.add_argument('--label',default='')
    p.add_argument('--noise-level',type=int,choices=[0,1,2],default=0);p.add_argument('--seed',type=int,default=0)
    p.add_argument('--view-conditions',type=Path);p.add_argument('--camera-roll-deg',type=float,default=0)
    args=p.parse_args()
    if args.fps<=0 or not np.isfinite(args.playback_speed) or args.playback_speed<=0:
        p.error('fps and playback-speed must be positive and finite')
    run(args)


if __name__=='__main__':main()
