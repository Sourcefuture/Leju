#!/usr/bin/env python3
"""Drag objects and edit table/robot/head placement in a local Kuavo scene."""
from __future__ import annotations
import argparse,json,math,os,tkinter as tk
from tkinter import messagebox
from pathlib import Path
os.environ.setdefault('MUJOCO_GL','egl')
import numpy as np
from prepare_kuavo_acceptance_scene import build,DEFAULT_SOURCE

SIZES={'bag_white':(.30,.20),'bag_green':(.30,.20),'bag_black':(.30,.20),'scale':(.23,.17),'material_box':(.40,.30)}
COLORS={'bag_white':'#eeeeeb','bag_green':'#218647','bag_black':'#252525','scale':'#8d9298','material_box':'#dddacb'}
DEFAULT_XY={'bag_white':[.25,.1],'bag_green':[.25,.3],'bag_black':[.25,.5],'scale':[.6,.3],'material_box':[.97,.3]}


def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--source-scene',type=Path,default=DEFAULT_SOURCE);p.add_argument('--config',type=Path,required=True)
    p.add_argument('--output',type=Path,required=True);a=p.parse_args()
    config=json.loads(a.config.read_text()) if a.config.is_file() else {}
    source_info=json.loads((a.source_scene.parent/'scene_info.json').read_text())
    items=config.setdefault('items',{})
    for name in SIZES:
        items.setdefault(name,{}).setdefault('xy_m',source_info['items'].get(name,{}).get('xy',DEFAULT_XY[name]))
    root=tk.Tk();root.title('Kuavo-5.2 scene editor | local simulation only')
    canvas=tk.Canvas(root,width=800,height=430,bg='#f2f2f2');canvas.grid(row=0,column=0,rowspan=20)
    vars={};selected=tk.StringVar(value='bag_white');status=tk.StringVar(value='Drag an object; Save scene rebuilds XML and assumed corners, not human calibration.')
    def slider(row,label,key,default,lo,hi,step):
        value=tk.DoubleVar(value=default);vars[key]=value
        tk.Label(root,text=label,anchor='w').grid(row=row,column=1,sticky='w')
        tk.Scale(root,from_=lo,to=hi,resolution=step,orient='horizontal',length=235,variable=value,command=lambda _:draw()).grid(row=row,column=2)
    delta=config.get('table_translation_m',[0,0,0]);robot=config.get('robot_translation_m',[0,0,0]);views=config.get('view_conditions',{})
    slider(0,'Near edge world X (m)','table_x',config.get('table_near_edge_x_m',.30),.20,.65,.005)
    slider(1,'Table world Y offset (m)','table_y',delta[1],-.15,.15,.005)
    slider(2,'Table yaw (deg)','table_yaw',config.get('table_yaw_deg',0),-12,12,.5)
    slider(3,'Robot world X offset (m)','robot_x',robot[0],-.08,.08,.005)
    slider(4,'Robot world Y offset (m)','robot_y',robot[1],-.15,.15,.005)
    slider(5,'Robot yaw (deg)','robot_yaw',config.get('robot_yaw_deg',0),-15,15,.5)
    slider(6,'Head yaw bias (deg)','head_yaw',views.get('head_yaw_bias_deg',0),-20,20,1)
    slider(7,'Head pitch bias (deg)','head_pitch',views.get('head_pitch_bias_deg',0),-8,15,1)
    slider(8,'Camera mount roll (deg)','roll',config.get('camera_mount_roll_deg',0),-30,30,1)
    slider(9,'Camera vertical FOV (deg)','fov',config.get('camera_fovy_deg',60),50,85,1)
    slider(10,'Near cloth drop (m)','cloth',config.get('near_cloth_drop_m',.12),.08,.4,.01)
    tk.OptionMenu(root,selected,*SIZES,command=lambda _:sync_yaw()).grid(row=11,column=1)
    yaw=tk.DoubleVar(value=items[selected.get()].get('yaw_deg',0))
    def set_yaw(_):
        item=items[selected.get()];item['yaw_deg']=yaw.get();w,h=SIZES[selected.get()]
        angle=math.radians(yaw.get());ex=(abs(math.cos(angle))*w+abs(math.sin(angle))*h)/2
        ey=(abs(math.sin(angle))*w+abs(math.cos(angle))*h)/2
        x,y=item['xy_m'];item['xy_m']=[float(np.clip(x,ex,1.2-ex)),float(np.clip(y,ey,.6-ey))]
        draw()
    tk.Scale(root,from_=-180,to=180,resolution=1,orient='horizontal',length=235,variable=yaw,command=set_yaw,label='Selected item yaw (deg)').grid(row=11,column=2)
    def sync_yaw():yaw.set(items[selected.get()].get('yaw_deg',0));draw()
    def polygon(name):
        x,y=items[name]['xy_m'];w,h=SIZES[name];angle=math.radians(items[name].get('yaw_deg',0));r=np.array([[math.cos(angle),-math.sin(angle)],[math.sin(angle),math.cos(angle)]])
        pts=np.array([[-w/2,-h/2],[w/2,-h/2],[w/2,h/2],[-w/2,h/2]])@r.T+[x,y]
        return pts
    def draw():
        if not root.winfo_exists():return
        canvas.delete('all');canvas.create_rectangle(20,20,780,400,fill='#ccb893',outline='#554a39',width=2)
        canvas.create_text(400,414,text='Near edge / robot side | table X right, table Y away | 1200 x 600 mm')
        for k,xy in zip('ABCD',[(0,0),(1.2,0),(1.2,.6),(0,.6)]):
            u=20+xy[0]*760/1.2;v=400-xy[1]*380/.6
            canvas.create_oval(u-5,v-5,u+5,v+5,fill='red');canvas.create_text(u+12,v-12,text=k,fill='red')
        for name in SIZES:
            pts=polygon(name);pixels=[c for x,y in pts for c in [20+x*760/1.2,400-y*380/.6]]
            canvas.create_polygon(pixels,fill=COLORS[name],outline='#e23a29' if selected.get()==name else '#444444',width=3 if selected.get()==name else 1,tags=name)
            x,y=items[name]['xy_m'];canvas.create_text(20+x*760/1.2,400-y*380/.6,text=name,fill='white' if name=='bag_black' else 'black',tags=name)
    dragging={'name':None}
    def press(event):
        names=[name for name in SIZES if name in canvas.gettags('current')]
        if names:dragging['name']=names[0];selected.set(names[0]);sync_yaw()
    def motion(event):
        name=dragging['name']
        if not name:return
        w,h=SIZES[name];angle=math.radians(items[name].get('yaw_deg',0))
        ex=(abs(math.cos(angle))*w+abs(math.sin(angle))*h)/2;ey=(abs(math.sin(angle))*w+abs(math.cos(angle))*h)/2
        x=np.clip((event.x-20)*1.2/760,ex,1.2-ex);y=np.clip((400-event.y)*.6/380,ey,.6-ey)
        items[name]['xy_m']=[round(float(x),4),round(float(y),4)];draw();status.set(f'{name}: table XY={items[name]["xy_m"]} m')
    canvas.bind('<Button-1>',press);canvas.bind('<B1-Motion>',motion);canvas.bind('<ButtonRelease-1>',lambda _:dragging.update(name=None))
    preview={'window':None,'photo':None}
    def collect():
        config.setdefault('table_yaw_pivot','center')
        config.update(table_near_edge_x_m=vars['table_x'].get(),table_translation_m=[0,vars['table_y'].get(),0],table_yaw_deg=vars['table_yaw'].get(),
            robot_translation_m=[vars['robot_x'].get(),vars['robot_y'].get(),0],robot_yaw_deg=vars['robot_yaw'].get(),
            camera_mount_roll_deg=vars['roll'].get(),camera_fovy_deg=vars['fov'].get(),near_cloth_drop_m=vars['cloth'].get(),hide_calibration_sites=True)
        config.setdefault('view_conditions',{}).update(head_yaw_bias_deg=vars['head_yaw'].get(),head_pitch_bias_deg=vars['head_pitch'].get())
        config['head_yaw_deg']=vars['head_yaw'].get();config['head_pitch_deg']=10+vars['head_pitch'].get()
        return config
    def save():
        try:
            c=collect();a.config.parent.mkdir(parents=True,exist_ok=True);a.config.write_text(json.dumps(c,indent=2)+'\n')
            scene=build(a.source_scene,a.output,c);status.set(f'Saved {scene}. Use a NEW state directory and recalibrate after changing placement.')
            return scene
        except Exception as error:messagebox.showerror('Scene build',str(error))
    def show_rgb():
        scene=save()
        if not scene:return
        try:
            from kuavo_sim_camera import KuavoSimCamera
            from kuavo_rgb_disturbance import apply_conditions
            from PIL import Image,ImageTk
            camera=KuavoSimCamera(scene,a.output/'editor_preview_state');rgb=camera.capture_rgb('head');camera.close()
            rgb=apply_conditions(rgb,np.random.default_rng(0),config.get('view_conditions',{}))
            if preview['window'] is None or not preview['window'].winfo_exists():preview['window']=tk.Toplevel(root)
            win=preview['window'];win.title('Actual simulated head RGB preview')
            for child in win.winfo_children():child.destroy()
            preview['photo']=ImageTk.PhotoImage(Image.fromarray(rgb));tk.Label(win,image=preview['photo']).pack()
        except Exception as error:messagebox.showerror('RGB preview',str(error))
    tk.Button(root,text='Save config and build scene',command=save).grid(row=12,column=1,columnspan=2)
    tk.Button(root,text='Preview head RGB',command=show_rgb).grid(row=13,column=1,columnspan=2)
    tk.Label(root,textvariable=status,wraplength=1050,anchor='w').grid(row=21,column=0,columnspan=3,sticky='w')
    draw();root.mainloop()


if __name__=='__main__':main()
