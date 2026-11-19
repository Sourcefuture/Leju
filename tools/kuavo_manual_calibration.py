#!/usr/bin/env python3
"""Explicit human corner marking from supplied XYZ or current simulated TCP.

No scene_info or simulated corner positions are used by this module. All manual
records require --confirm and retain their provenance. Run only in local sim.
"""
from __future__ import annotations
import argparse, hashlib, json
from pathlib import Path
import numpy as np
from kuavo_table_vision import FourPointCalibration, VisionError
from kuavo_planar_task import atomic_json, load_json
from kuavo_arm_controller import now_iso


TABLE_XY = dict(zip('ABCD',[[0,0],[1.2,0],[1.2,.6],[0,.6]]))


def measurement_digest(session,key):
    return hashlib.sha256(json.dumps(session.get(key,{}),sort_keys=True,separators=(',',':')).encode()).hexdigest()


class ManualCalibration:
    def __init__(self,scene,state_dir):
        self.scene=Path(scene).resolve();self.state_dir=Path(state_dir).resolve()
        self.path=self.state_dir/'manual_calibration.json'
        self.fingerprint=hashlib.sha256(self.scene.read_bytes()).hexdigest()
        self.session=load_json(self.path,{'schema_version':1,'scene_sha256':self.fingerprint,'points':{}})
        if self.session['scene_sha256']!=self.fingerprint:
            raise ValueError('Scene changed: use a new calibration directory')

    def mark(self,point,xyz,clearance,confirmed,source,pixel=None,image=None,arm14=None):
        if not confirmed: raise ValueError('Inspect the corner and supply --confirm explicitly')
        values=np.asarray(xyz,float)
        if values.shape!=(3,) or not np.isfinite(values).all() or not np.isfinite(clearance):
            raise ValueError('Finite XYZ and clearance in metres are required')
        if not 0<=clearance<=.5: raise ValueError('Clearance must be 0..0.5 m')
        record={'point':point,'table_xy_m':TABLE_XY[point],'robot_xyz_m':values.tolist(),
                'clearance_m':float(clearance),'operator_confirmed':True,
                'provenance':source,'recorded_at':now_iso()}
        if pixel is not None:
            if len(pixel)!=2 or not np.isfinite(pixel).all(): raise ValueError('Finite pixel x,y required')
            if image is None: raise ValueError('--pixel requires --image so the camera frame is identifiable')
            record.update(image_pixel_xy=list(map(float,pixel)),image=str(Path(image).resolve()),
                          image_sha256=hashlib.sha256(Path(image).read_bytes()).hexdigest())
        if arm14 is not None: record['arm14_rad']=list(map(float,arm14))
        previous=self.session['points'].get(point)
        if previous:
            self.session.setdefault('revisions',[]).append(previous)
        self.session['points'][point]=record;self.session.pop('fit',None)
        atomic_json(self.path,self.session);return record

    def fit(self,output):
        if set(self.session['points'])!=set('ABCD'): raise ValueError('Manually mark all four corners A, B, C and D')
        points=self.session['points']
        if not all(p.get('operator_confirmed') for p in points.values()): raise ValueError('All four points need human confirmation')
        clearances=[points[k]['clearance_m'] for k in 'ABCD']
        if np.ptp(clearances)>.001: raise ValueError('Use the same clearance at all four corners')
        cal=FourPointCalibration({k:points[k]['robot_xyz_m'] for k in 'ABCD'},float(np.mean(clearances)))
        if cal.normal[2]<.999: raise ValueError('Fixed palm-down task currently requires a horizontal table')
        result={'reference_frame':'mujoco_world','clearance_m':cal.clearance,
                'corners_robot':{k:points[k]['robot_xyz_m'] for k in 'ABCD'},
                'provenance':'four explicitly confirmed manual measurements; see manual_calibration.json',
                'manual_measurements_key':'points','manual_measurements_sha256':measurement_digest(self.session,'points'),
                'manual_session_file':str(self.path.resolve()),'max_residual_mm':cal.max_residual_mm}
        atomic_json(output,result)
        self.session['fit']={'affine_world_from_xy':cal.coeff.tolist(),'normal_world':cal.normal.tolist(),
                             'max_residual_mm':cal.max_residual_mm,'export':str(Path(output).resolve())}
        atomic_json(self.path,self.session);return result

    def mark_reference(self,name,table_xy,xyz,clearance,confirmed,source):
        if not confirmed:raise ValueError('Inspect the reference and explicitly confirm')
        xy=np.asarray(table_xy,float);values=np.asarray(xyz,float)
        if xy.shape!=(2,) or values.shape!=(3,) or not np.isfinite(np.r_[xy,values,clearance]).all():raise ValueError('Finite table XY, robot XYZ and clearance required')
        if np.any(xy<0) or np.any(xy>[1.2,.6]) or not 0<=clearance<=.5:raise ValueError('Reference outside the table or invalid clearance')
        record={'name':name,'table_xy_m':xy.tolist(),'robot_xyz_m':values.tolist(),'clearance_m':clearance,
                'operator_confirmed':True,'provenance':source,'recorded_at':now_iso()}
        references=self.session.setdefault('references',{})
        if name in references:self.session.setdefault('revisions',[]).append({'kind':'reference','previous':references[name]})
        references[name]=record;self.session.pop('fit',None)
        atomic_json(self.path,self.session);return record

    def fit_references(self,output):
        refs=list(self.session.get('references',{}).values())
        if len(refs)<4 or not all(r.get('operator_confirmed') for r in refs):raise ValueError('At least four confirmed, reachable internal references are required')
        xy=np.array([r['table_xy_m'] for r in refs]);xyz=np.array([r['robot_xyz_m'] for r in refs]);h=np.array([r['clearance_m'] for r in refs])
        if np.ptp(h)>.001:raise ValueError('Use the same TCP clearance for all internal references')
        if np.ptp(xy[:,0])<.4 or np.ptp(xy[:,1])<.2:raise ValueError('Spread references by at least .4 m in X and .2 m in Y')
        design=np.c_[np.ones(len(refs)),xy]
        if np.linalg.matrix_rank(design)<3:raise ValueError('Reference points are collinear')
        coeff=np.linalg.lstsq(design,xyz,rcond=None)[0];error=np.linalg.norm(design@coeff-xyz,axis=1)
        if max(error)>.005:raise ValueError('Reference fit residual exceeds 5 mm')
        corners={k:(np.r_[1,v]@coeff).tolist() for k,v in TABLE_XY.items()}
        cal=FourPointCalibration(corners,float(np.mean(h)))
        if cal.normal[2]<.999:raise ValueError('Table must be horizontal for the fixed-palm-down task')
        gain={k:float(np.sum(abs(np.r_[1,v]@np.linalg.pinv(design)))) for k,v in TABLE_XY.items()}
        result={'reference_frame':'mujoco_world','clearance_m':cal.clearance,'corners_robot':corners,
                'provenance':'corners derived from at least four confirmed internal measured reference points; NOT four arm-touched corners',
                'manual_measurements_key':'references','manual_measurements_sha256':measurement_digest(self.session,'references'),
                'reference_fit_max_residual_mm':float(max(error)*1000),'corner_error_amplification_l1':gain,
                'manual_session_file':str(self.path),'warning':'Extrapolation amplifies measurement error; zero fit residual is not zero coordinate error'}
        atomic_json(output,result);self.session['fit']={'method':'internal_reference_extrapolation','export':str(output),'corner_error_amplification_l1':gain}
        atomic_json(self.path,self.session);return result


def click_gui(session,image,clearance):
    import tkinter as tk
    from tkinter import messagebox
    from PIL import Image,ImageTk
    original=Image.open(image).convert('RGB');root=tk.Tk();root.title('Kuavo manual table corners | A near-left, B near-right, C far-right, D far-left')
    factor=min(1.,1000/original.width,650/original.height)
    display=original.resize((round(original.width*factor),round(original.height*factor)))
    picture=ImageTk.PhotoImage(display);canvas=tk.Canvas(root,width=display.width,height=display.height)
    canvas.grid(row=0,column=0,columnspan=5);canvas.create_image(0,0,anchor='nw',image=picture)
    selected=tk.StringVar(value='A');clicked={};entries=[]
    tk.Label(root,text='Corner').grid(row=1,column=0);tk.OptionMenu(root,selected,*'ABCD').grid(row=1,column=1)
    for i,label in enumerate(['Robot X (m)','Robot Y (m)','Robot Z (m)']):
        tk.Label(root,text=label).grid(row=2,column=i);e=tk.Entry(root,width=16);e.grid(row=3,column=i);entries.append(e)
    status=tk.StringVar(value='Click a visible corner, enter measured robot XYZ, then confirm. No automatic true coordinates.')
    tk.Label(root,textvariable=status,wraplength=900).grid(row=5,column=0,columnspan=5)
    def click(event):
        k=selected.get();clicked[k]=[event.x/factor,event.y/factor]
        canvas.delete('point_'+k);canvas.create_oval(event.x-5,event.y-5,event.x+5,event.y+5,outline='red',width=2,tags='point_'+k)
        canvas.create_text(event.x+12,event.y-12,text=k,fill='red',tags='point_'+k)
        status.set(f'{k}: pixel {clicked[k]}. Enter XYZ measured in the robot/world frame.')
    def save():
        try:
            k=selected.get()
            if k not in clicked: raise ValueError('Click this corner first')
            session.mark(k,[float(e.get()) for e in entries],clearance,True,'operator_entered_XYZ_and_RGB_click',clicked[k],image)
            status.set(f'Saved {k}. Recorded corners: {", ".join(sorted(session.session["points"]))}')
            remaining=[k for k in 'ABCD' if k not in session.session['points']]
            if remaining:selected.set(remaining[0])
        except Exception as error: messagebox.showerror('Calibration',str(error))
    def export():
        try:
            result=session.fit(session.state_dir/'manual_corners.json');status.set('Exported '+str(session.state_dir/'manual_corners.json'))
        except Exception as error:messagebox.showerror('Calibration',str(error))
    canvas.bind('<Button-1>',click)
    tk.Button(root,text='Confirm and record point',command=save).grid(row=4,column=0,columnspan=2)
    tk.Button(root,text='Fit and export four corners',command=export).grid(row=4,column=2,columnspan=2)
    root.mainloop()


def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--scene',type=Path,required=True);p.add_argument('--state-dir',type=Path,required=True)
    sub=p.add_subparsers(dest='command',required=True)
    a=sub.add_parser('set-point');a.add_argument('point',choices=list('ABCD'));a.add_argument('--xyz',type=float,nargs=3,required=True)
    a.add_argument('--clearance-m',type=float,default=0);a.add_argument('--confirm',action='store_true')
    a.add_argument('--pixel',type=float,nargs=2);a.add_argument('--image',type=Path)
    a=sub.add_parser('record-tcp');a.add_argument('point',choices=list('ABCD'));a.add_argument('--arm',choices=['left','right'],required=True)
    a.add_argument('--clearance-m',type=float,default=0);a.add_argument('--confirm',action='store_true')
    a=sub.add_parser('fit');a.add_argument('--output',type=Path)
    a=sub.add_parser('set-reference');a.add_argument('name');a.add_argument('--table-xy',type=float,nargs=2,required=True)
    coords=a.add_mutually_exclusive_group(required=True);coords.add_argument('--xyz',type=float,nargs=3);coords.add_argument('--arm',choices=['left','right'])
    a.add_argument('--clearance-m',type=float,default=0);a.add_argument('--confirm',action='store_true')
    a=sub.add_parser('fit-references');a.add_argument('--output',type=Path)
    sub.add_parser('status')
    a=sub.add_parser('gui');a.add_argument('--image',type=Path,required=True);a.add_argument('--clearance-m',type=float,default=0)
    args=p.parse_args();session=ManualCalibration(args.scene,args.state_dir)
    if args.command=='set-point':
        result=session.mark(args.point,args.xyz,args.clearance_m,args.confirm,'operator_entered_robot_XYZ',args.pixel,args.image)
    elif args.command=='record-tcp':
        from kuavo_planar_task import PlanarTask
        task=PlanarTask(args.scene,args.state_dir)
        result=session.mark(args.point,task.arm._eef(args.arm)[0],args.clearance_m,args.confirm,
                            'operator_confirmed_current_simulated_'+args.arm+'_TCP',arm14=task.arm._arm14_rad())
    elif args.command=='fit':result=session.fit(args.output or args.state_dir/'manual_corners.json')
    elif args.command=='set-reference':
        xyz=args.xyz;source='operator_entered_internal_reference_XYZ'
        if args.arm:
            from kuavo_planar_task import PlanarTask
            task=PlanarTask(args.scene,args.state_dir);xyz=task.arm._eef(args.arm)[0];source='operator_confirmed_current_simulated_'+args.arm+'_TCP'
        result=session.mark_reference(args.name,args.table_xy,xyz,args.clearance_m,args.confirm,source)
    elif args.command=='fit-references':result=session.fit_references(args.output or args.state_dir/'manual_corners.json')
    elif args.command=='gui':click_gui(session,args.image,args.clearance_m);return
    else: result=session.session
    print(json.dumps(result,ensure_ascii=False,indent=2))


if __name__=='__main__':main()
