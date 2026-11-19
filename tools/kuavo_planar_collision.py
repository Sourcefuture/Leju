#!/usr/bin/env python3
"""Collision geometry reconstructed from manual calibration and RGB detections."""
from __future__ import annotations
import xml.etree.ElementTree as ET
import mujoco,numpy as np


class ObservedCollisionWorld:
    def __init__(self, robot_scene, calibration, observation):
        # Strip every real scene object. Only the known robot model is retained.
        xml=ET.parse(robot_scene).getroot();world=xml.find('worldbody')
        for node in list(world):
            if node.tag!='body' or node.get('name')!='base_link':world.remove(node)
        x,y=calibration.coeff[1],calibration.coeff[2]
        x=x/np.linalg.norm(x);y=y-x*(x@y);y=y/np.linalg.norm(y);z=np.cross(x,y)
        matrix=np.c_[x,y,z];quat=np.zeros(4);mujoco.mju_mat2Quat(quat,matrix.ravel())
        origin=calibration.robot_xyz([0,0],0)
        body=ET.SubElement(world,'body',name='observed_table_frame',pos=' '.join(map(str,origin)),quat=' '.join(map(str,quat)))
        def obstacle(name,xy,size,height,yaw=0):
            ET.SubElement(body,'geom',name='observed_'+name,type='box',
                pos=' '.join(map(str,[*xy,height/2])),size=' '.join(map(str,[size[0]/2,size[1]/2,height/2])),
                contype='8',conaffinity='8',quat=' '.join(map(str,[np.cos(yaw/2),0,0,np.sin(yaw/2)])))
        obstacle('table',[.6,.3],[1.2,.6],.03)
        # Table extends below its calibrated surface.
        body.find("geom[@name='observed_table']").set('pos','.6 .3 -.015')
        # Known table construction, expressed in the manually calibrated frame.
        # Floor Z=0 is this simulation's robot support-plane convention.
        table_height=float(origin[2])
        if table_height>.08:
            for index,xy in enumerate(((.055,.055),(1.145,.055),(.055,.545),(1.145,.545))):
                obstacle('table_leg_'+str(index),xy,[.05,.05],table_height-.031)
                body.find(f"geom[@name='observed_table_leg_{index}']").set('pos',
                    ' '.join(map(str,[*xy,-(table_height+.031)/2])))
        for name,det in observation['detections'].items():
            yaw=float(det.get('yaw_table_rad',0));rotation=np.array([[np.cos(yaw),-np.sin(yaw)],[np.sin(yaw),np.cos(yaw)]])
            dims=([.3,.2] if name.startswith('bag_') else ([.23,.17] if name=='scale' else [.4,.3]))
            if name=='material_box':
                # Hollow container: a solid proxy spuriously blocks clear air
                # above its interior. Wall thickness is the declared 10 mm assumption.
                x,y=det['table_xy_m'];h=det['height_m'];th=.01
                wall_margin=.008  # image-derived centre uncertainty, not a new sensor input
                obstacle('bin_bottom',[x,y],dims,th,yaw)
                for tag,pos,size in [('left',[x-.195,y],[th,.30]),('right',[x+.195,y],[th,.30]),
                                     ('near',[x,y-.145],[.38,th]),('far',[x,y+.145],[.38,th])]:
                    inflated=[size[0]+2*wall_margin,size[1]+2*wall_margin]
                    rotated=np.array([x,y])+rotation@(np.array(pos)-[x,y])
                    obstacle('bin_'+tag,rotated,inflated,h+wall_margin,yaw)
                obstacle('bin_fill',[x,y],[.38,.28],.12,yaw)
            elif name.startswith('bag_'):
                # height_m is the observed top above the table, not object
                # thickness. A parcel on a 20 mm scale remains 12 mm thick.
                obstacle(name,det['table_xy_m'],dims,.012,yaw)
                body.find(f"geom[@name='observed_{name}']").set('pos',
                    ' '.join(map(str,[*det['table_xy_m'],det['height_m']-.006])))
            else:obstacle(name,det['table_xy_m'],dims,det['height_m'],yaw)
        proxy=ET.SubElement(world,'body',name='held_parcel_proxy',pos='0 0 -10',quat=' '.join(map(str,quat)))
        ET.SubElement(proxy,'geom',name='held_parcel_shape',type='box',size='.15 .1 .006',
                      contype='0',conaffinity='0',group='4')
        self.model=mujoco.MjModel.from_xml_string(ET.tostring(xml,encoding='unicode'))
        self.data=mujoco.MjData(self.model)
        if self.model.nkey:mujoco.mj_resetDataKeyframe(self.model,self.data,0)
        self.addresses=[self.model.joint(f'zarm_{s}{i}_joint').qposadr[0] for s in ('l','r') for i in range(1,8)]
        self.robot_geoms=[];self.obstacles=[]
        self.held=None
        self.parcel_yaws={name:float(det.get('yaw_table_rad',0)) for name,det in observation['detections'].items() if name.startswith('bag_')}
        self.table_quat=quat.copy()
        self.escape_contacts={};self.escape_origin=None
        self.held_body=self.model.body('held_parcel_proxy').id
        self.held_geom=self.model.geom('held_parcel_shape').id
        for i in range(self.model.ngeom):
            name=self.model.geom(i).name or ''
            bodyname=self.model.body(self.model.geom_bodyid[i]).name or ''
            if bodyname.startswith('zarm_') and self.model.geom_group[i]==3:self.robot_geoms.append(i)
            if name.startswith('observed_'):self.obstacles.append(i)
        all_robot=[i for i in range(self.model.ngeom) if self.model.geom_group[i]==3
                   and not (self.model.geom(i).name or '').startswith('observed_')]
        self.fixed_body_geoms=[i for i in all_robot if i not in self.robot_geoms]
        self.self_pairs=[]
        for arm in self.robot_geoms:
            aname=self.model.body(self.model.geom_bodyid[arm]).name
            for other in all_robot:
                if other==arm:continue
                bname=self.model.body(self.model.geom_bodyid[other]).name
                if bname.startswith('zarm_'):
                    if aname[:6]==bname[:6] and abs(int(aname[6])-int(bname[6]))<=2:continue
                    if arm>other:continue
                # Shoulder mounting primitives touch the torso by design.
                if aname.endswith('1_link') and bname in ('waist_yaw','base_link'):continue
                self.self_pairs.append((arm,other))
        self.robot_obstacle_pairs=np.array([(r,o) for r in all_robot for o in self.obstacles],dtype=int).reshape(-1,2)
        self.self_pair_indices=np.array(self.self_pairs,dtype=int).reshape(-1,2)

    def pair_distances(self,pairs,limit):
        """Conservative bounding spheres skip only provably distant pairs.

        Residual length stays fixed for least_squares; near pairs still use the
        exact MuJoCo primitive distance. geom_rbound includes mesh scale.
        """
        pairs=np.asarray(pairs,dtype=int).reshape(-1,2)
        values=np.full(len(pairs),float(limit))
        if not len(pairs):return values
        delta=self.data.geom_xpos[pairs[:,0]]-self.data.geom_xpos[pairs[:,1]]
        radius=self.model.geom_rbound[pairs[:,0]]+self.model.geom_rbound[pairs[:,1]]+limit
        near=np.flatnonzero(np.einsum('ij,ij->i',delta,delta)<=radius*radius)
        for index in near:
            first,second=pairs[index]
            values[index]=mujoco.mj_geomDistance(self.model,self.data,int(first),int(second),limit,None)
        return values

    def set_held(self,name=None,side=None,tcp_local=None,offset_world=None,arm14_rad=None):
        self.held=(name,side,np.asarray(tcp_local),np.asarray(offset_world)) if name else None
        if name:
            yaw=self.parcel_yaws.get(name,0);q=np.zeros(4)
            mujoco.mju_mulQuat(q,self.table_quat,np.array([np.cos(yaw/2),0,0,np.sin(yaw/2)]))
            self.model.body_quat[self.held_body]=q
        self.escape_contacts={};self.escape_origin=None
        if name and arm14_rad is not None:
            initial=self.held_check(arm14_rad)
            self.escape_origin=self.model.body_pos[self.held_body].copy()
            # RGB uncertainty may make touching flat bundles overlap slightly.
            # Permit only escape from an already existing small neighbour contact,
            # within 3 mm horizontally and the first 20 mm of vertical lift.
            for hit in initial:
                if hit['obstacle'].startswith('observed_bag_') and hit['penetration_mm']<=8:
                    self.escape_contacts[hit['obstacle']]=hit['penetration_mm']/1000+.0005
        return dict(self.escape_contacts)

    def relocate_bag(self,name,base_world):
        try:ident=self.model.geom('observed_'+name).id
        except KeyError:return
        body=self.model.body('observed_table_frame').id
        rotation=self.data.xmat[body].reshape(3,3)
        self.model.geom_pos[ident]=rotation.T@(np.asarray(base_world)-self.data.xpos[body])+[0,0,.006]

    def set_packed_count(self,count):
        ident=self.model.geom('observed_bin_fill').id
        height=.12+count*.012
        self.model.geom_size[ident,2]=height/2;self.model.geom_pos[ident,2]=height/2
        self.model.geom_rbound[ident]=np.linalg.norm(self.model.geom_size[ident])

    def held_check(self,arm14_rad,tolerance_m=.002):
        if not self.held:return []
        name,side,tcp_local,offset=self.held
        self.data.qpos[self.addresses]=arm14_rad;mujoco.mj_kinematics(self.model,self.data)
        wrist=self.model.body('zarm_'+('l' if side=='left' else 'r')+'7_link').id
        normal=self.data.xmat[self.model.body('observed_table_frame').id].reshape(3,3)[:,2]
        self.model.body_pos[self.held_body]=(self.data.xpos[wrist]
            +self.data.xmat[wrist].reshape(3,3)@tcp_local+offset+normal*.006)
        mujoco.mj_kinematics(self.model,self.data)
        result=[]
        for obstacle in self.obstacles:
            if self.model.geom(obstacle).name=='observed_'+name:continue
            distance=mujoco.mj_geomDistance(self.model,self.data,self.held_geom,obstacle,.01,None)
            label=self.model.geom(obstacle).name
            if label in self.escape_contacts and self.escape_origin is not None:
                delta=self.model.body_pos[self.held_body]-self.escape_origin
                if np.linalg.norm(delta[:2])<=.003 and -.001<=delta[2]<=.02 and distance>=-self.escape_contacts[label]:continue
            if distance < -tolerance_m:
                result.append({'robot_body':'held_'+name,'obstacle':self.model.geom(obstacle).name,
                               'penetration_mm':-float(distance)*1000})
        return result

    def self_check(self,arm14_rad,tolerance_m=.002,refresh=True):
        if refresh:
            self.data.qpos[self.addresses]=arm14_rad;mujoco.mj_kinematics(self.model,self.data)
        result=[]
        for (first,second),distance in zip(self.self_pair_indices,self.pair_distances(self.self_pair_indices,.01)):
            if distance < -tolerance_m:
                result.append({'robot_body':self.model.body(self.model.geom_bodyid[first]).name,
                               'other_robot_body':self.model.body(self.model.geom_bodyid[second]).name,
                               'penetration_mm':-float(distance)*1000})
        return result

    def check(self, arm14_rad, tolerance_m=.002):
        self.data.qpos[self.addresses]=arm14_rad;mujoco.mj_kinematics(self.model,self.data)
        collisions=[]
        for (robot,obstacle),distance in zip(self.robot_obstacle_pairs,self.pair_distances(self.robot_obstacle_pairs,.01)):
            if distance < -tolerance_m:
                collisions.append({'robot_body':self.model.body(self.model.geom_bodyid[robot]).name,
                                   'obstacle':self.model.geom(obstacle).name,'penetration_mm':-float(distance)*1000})
        return collisions+self.self_check(arm14_rad,tolerance_m,refresh=False)+self.held_check(arm14_rad,tolerance_m)

    def penalties(self, arm14_rad, side, clearance_m=.003):
        self.data.qpos[self.addresses]=arm14_rad;mujoco.mj_kinematics(self.model,self.data)
        prefix='zarm_l' if side=='left' else 'zarm_r'
        pairs=[(r,o) for r,o in self.robot_obstacle_pairs if self.model.body(self.model.geom_bodyid[r]).name.startswith(prefix)]
        for first,second in self.self_pair_indices:
            names=[self.model.body(self.model.geom_bodyid[i]).name for i in (first,second)]
            if not any(n.startswith(prefix) for n in names):continue
            pairs.append((first,second))
        return np.maximum(0.,clearance_m-self.pair_distances(pairs,clearance_m+.001))
