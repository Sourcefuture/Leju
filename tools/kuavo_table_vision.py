#!/usr/bin/env python3
"""RGB-only planar localization. This module deliberately has no simulator imports."""
from __future__ import annotations

import cv2
import numpy as np

TABLE_SIZE = (1.2, .6)
TAG_CENTERS = {0:(.05,.05), 1:(1.15,.05), 2:(1.15,.55), 3:(.05,.55),
               4:(.60,.05), 5:(.60,.55)}
TAG_SIZE = .06


class VisionError(ValueError):
    pass


class FourPointCalibration:
    def __init__(self, corners_robot, clearance_m=0):
        xy=np.array([[0,0],[1.2,0],[1.2,.6],[0,.6]],float)
        world=np.array([corners_robot[k] for k in 'ABCD'],float)
        if world.shape!=(4,3) or not np.isfinite(world).all():raise VisionError('Four finite robot XYZ points are required')
        design=np.c_[np.ones(4),xy]
        self.coeff=np.linalg.lstsq(design,world,rcond=None)[0]
        residual=np.linalg.norm(design@self.coeff-world,axis=1)
        if max(residual)>.005:raise VisionError('Manual corner calibration residual exceeds 5 mm')
        x,y=self.coeff[1:]
        if abs(np.linalg.norm(x)-1)>.025 or abs(np.linalg.norm(y)-1)>.025 or abs(x@y)>.025:
            raise VisionError('Manual calibration disagrees with the 120 x 60 cm rectangle')
        self.normal=np.cross(x,y);self.normal/=np.linalg.norm(self.normal)
        if self.normal[2]<0:self.normal=-self.normal
        self.clearance=float(clearance_m)
        if not np.isfinite(self.clearance):raise VisionError('Calibration clearance must be finite')
        self.max_residual_mm=float(max(residual)*1000)

    def robot_xyz(self, xy, height=0):
        return np.r_[1,xy]@self.coeff+self.normal*(height-self.clearance)


def project(points, homography):
    points=np.asarray(points,dtype=float).reshape(-1,1,2)
    return cv2.perspectiveTransform(points,homography).reshape(-1,2)


class TableVision:
    """Only RGB, supplied manual coordinates, and known physical dimensions enter."""
    def __init__(self, calibration, allow_object_tags=True,table_markers=None,allow_red_fallback=False):
        self.calibration=calibration
        self.allow_object_tags=allow_object_tags
        self.allow_red_fallback=allow_red_fallback
        self.parcel_yaw_priors={}
        self.tag_centers={int(k):tuple(v) for k,v in (TAG_CENTERS if table_markers is None else table_markers).items()}
        if any(k<0 or k>=50 or k in [10,11,12,13] for k in self.tag_centers):
            raise VisionError('Table markers must use dictionary IDs 0..49, excluding device IDs 10..13')
        if len(self.tag_centers)<2 or any(len(v)!=2 or not np.isfinite(v).all() or
                not (0<=v[0]<=1.2 and 0<=v[1]<=.6) for v in self.tag_centers.values()):
            raise VisionError('Printed table marker layout must contain finite points inside 120 x 60 cm')
        p=cv2.aruco.DetectorParameters();p.cornerRefinementMethod=cv2.aruco.CORNER_REFINE_SUBPIX
        p.cornerRefinementWinSize=3
        self.detector=cv2.aruco.ArucoDetector(cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50),p)

    def table_homography(self, rgb):
        gray=cv2.cvtColor(rgb,cv2.COLOR_RGB2GRAY)
        corners,ids,_=self.detector.detectMarkers(gray)
        tags={} if ids is None else {int(i):p.reshape(4,2) for i,p in zip(ids[:,0],corners)}
        present=[i for i in self.tag_centers if i in tags]
        spread=max((np.linalg.norm(np.array(self.tag_centers[a])-self.tag_centers[b]) for a in present for b in present),default=0)
        if len(present)>=2 and spread>.45:
            image=[];table=[]
            for i in present:
                x,y=self.tag_centers[i];r=TAG_SIZE/2
                image.extend(tags[i]);table.extend([(x-r,y+r),(x+r,y+r),(x+r,y-r),(x-r,y-r)])
            h,inliers=cv2.findHomography(np.array(image),np.array(table),0)
            if h is None:raise VisionError('Fiducial homography failed')
            fit=project(table,np.linalg.inv(h));pixel_error=np.linalg.norm(fit-np.array(image),axis=1)
            if np.median(pixel_error)>2.5:raise VisionError('Unstable table homography')
            mode='identified_table_fiducials';quality=float(np.median(pixel_error))
        else:
            if not self.allow_red_fallback:
                raise VisionError('Need at least two widely spaced identified table fiducials; unlabeled red points are not an orientation reference')
            # Marker-free-table baseline: existing four red corner dots, modest roll.
            hsv=cv2.cvtColor(rgb,cv2.COLOR_RGB2HSV)
            red=((hsv[:,:,0]<10)|(hsv[:,:,0]>170))&(hsv[:,:,1]>120)&(hsv[:,:,2]>70)
            contours,_=cv2.findContours(red.astype(np.uint8)*255,cv2.RETR_EXTERNAL,cv2.CHAIN_APPROX_SIMPLE)
            candidates=[]
            for contour in contours:
                area=cv2.contourArea(contour)
                if not 3<=area<=500:continue
                moments=cv2.moments(contour)
                if moments['m00']:candidates.append([moments['m10']/moments['m00'],moments['m01']/moments['m00']])
            if len(candidates)!=4:raise VisionError(f'Need two widely spaced identified tags or exactly four visible red corners; found {len(candidates)} red candidates')
            pts=np.array(candidates,np.float32)
            sums=pts.sum(1);diff=pts[:,0]-pts[:,1]
            idx=[np.argmin(diff),np.argmax(sums),np.argmax(diff),np.argmin(sums)]
            if len(set(idx))!=4:raise VisionError('Ambiguous red corner ordering')
            ordered=pts[idx]
            h=cv2.getPerspectiveTransform(ordered,np.array([[0,0],[1.2,0],[1.2,.6],[0,.6]],np.float32))
            mode='red_corners_orientation_assumed';quality=None
        table_in_image=project([[0,0],[1.2,0],[1.2,.6],[0,.6]],np.linalg.inv(h))
        area=abs(cv2.contourArea(table_in_image.astype(np.float32)))
        if area<rgb.shape[0]*rgb.shape[1]*.04:raise VisionError('Table covers too few pixels')
        return h,tags,{'mode':mode,'median_reprojection_px':quality,'visible_ids':present,'table_pixel_area':area}

    def estimate_camera_from_plane(self, h, shape):
        """Optional self-calibration: centered square pixels, orthogonal table axes.

        No camera matrices or simulated camera positions are consumed. Nearly
        fronto-parallel images are ill-conditioned and return no height estimate.
        """
        height,width=shape[:2];cx=(width-1)/2;cy=(height-1)/2
        inverse=np.linalg.inv(h);a,b=inverse[:,0],inverse[:,1]
        axy=a[:2]-np.array([cx,cy])*a[2];bxy=b[:2]-np.array([cx,cy])*b[2]
        numer=np.array([-axy@bxy, -(axy@axy-bxy@bxy)])
        denom=np.array([a[2]*b[2],a[2]**2-b[2]**2])
        if np.linalg.norm(denom)<1e-9:return None
        focal2=float(numer@denom/(denom@denom))
        if not (width*.20)**2<focal2<(width*4)**2:return None
        focal=np.sqrt(focal2);k=np.array([[focal,0,cx],[0,focal,cy],[0,0,1]])
        decomposed=np.linalg.inv(k)@inverse
        scale=2/(np.linalg.norm(decomposed[:,0])+np.linalg.norm(decomposed[:,1]))
        if decomposed[2,2]*scale<0:scale=-scale
        r1,r2=decomposed[:,0]*scale,decomposed[:,1]*scale
        # Near frontal views carry too little projective information to infer
        # focal length. A noisy positive focal solution can overcorrect a flat
        # parcel by more than its actual parallax. Keep the image-plane estimate
        # instead of applying an ill-conditioned inferred camera model.
        if np.linalg.norm([r1[2],r2[2]])<.15:return None
        r=np.c_[r1,r2,np.cross(r1,r2)];u,_,vt=np.linalg.svd(r);r=u@vt
        t=decomposed[:,2]*scale;camera=-r.T@t
        if not .35<camera[2]<3.5:return None
        return {'origin_table_m':camera.tolist(),'focal_px':float(focal),'assumption':'centered square-pixel camera; inferred from table homography'}

    def detect(self, rgb, required=None, on_scale=None):
        required=set(required or ['bag_white','bag_green','bag_black','scale','material_box'])
        on_scale=set(on_scale or [])
        if rgb.ndim!=3 or rgb.shape[2]!=3 or rgb.dtype!=np.uint8:raise VisionError('Expected RGB uint8 HxWx3')
        h,tags,quality=self.table_homography(rgb)
        # Fiducial white cells are a photometric reference, available only from RGB.
        references=[]
        for vertices in tags.values():
            mask=np.zeros(rgb.shape[:2],np.uint8)
            cv2.fillConvexPoly(mask,vertices.round().astype(np.int32),1)
            pixels=rgb[mask>0]
            if len(pixels)>20:references.append(np.percentile(pixels,90,axis=0))
        if references:
            white=np.median(references,axis=0)
            normalized=np.clip(rgb.astype(float)*255/np.maximum(white,30),0,255).astype(np.uint8)
        else:normalized=rgb
        normalized=cv2.GaussianBlur(normalized,(3,3),.6)
        warp=cv2.warpPerspective(normalized,np.diag([1000,1000,1])@h,(1200,600))
        hsv=cv2.cvtColor(warp,cv2.COLOR_RGB2HSV)
        lab=cv2.cvtColor(warp,cv2.COLOR_RGB2LAB).astype(float)
        chroma=np.linalg.norm(lab[:,:,1:]-128,axis=-1)
        masks={
            'bag_green':(hsv[:,:,0]>35)&(hsv[:,:,0]<100)&(hsv[:,:,1]>65)&(hsv[:,:,2]>30),
            # The previous L<180 also classified arm shadows, the dark scale
            # chassis and large unobserved image regions as a black parcel.
            'bag_black':(lab[:,:,0]<70)&(chroma<22),
            'neutral':(hsv[:,:,1]<30)&(hsv[:,:,2]>120),
        }
        # Known printed corner-label footprints can touch a dark parcel. Their
        # black cells must not become part of the parcel contour, even when the
        # label itself is partially hidden and its ID cannot be decoded.
        for x,y in self.tag_centers.values():
            u,v=round(x*1000),round(y*1000)
            for mask in masks.values():
                mask[max(0,v-40):min(600,v+40),max(0,u-40):min(1200,u+40)]=False
        detections={};camera=self.estimate_camera_from_plane(h,rgb.shape)
        def rectangles(mask):
            cleaned=cv2.morphologyEx(mask.astype(np.uint8)*255,cv2.MORPH_OPEN,np.ones((13,13),np.uint8))
            cleaned=cv2.morphologyEx(cleaned,cv2.MORPH_CLOSE,np.ones((5,5),np.uint8))
            contours,_=cv2.findContours(cleaned,cv2.RETR_EXTERNAL,cv2.CHAIN_APPROX_SIMPLE)
            result=[]
            for contour in contours:
                if cv2.contourArea(contour)<1200:continue
                rect=cv2.minAreaRect(contour)
                xy=np.array(rect[0])/1000;size=np.array(rect[1])/1000
                angle=np.radians(rect[2]+(90 if size[1]>size[0] else 0))
                angle=(angle+np.pi/2)%np.pi-np.pi/2
                result.append((xy,size,float(cv2.contourArea(contour))/1e6,float(angle),contour.reshape(-1,2)/1000))
            return result
        def put(name,xy,height,source,uncertainty_mm,yaw_rad=0):
            xy=np.asarray(xy)
            if not (np.all(xy>=[0,0]) and np.all(xy<=[1.2,.6])):raise VisionError(f'{name} is outside the table')
            detections[name]={'table_xy_m':xy.tolist(),'robot_xyz_m':self.calibration.robot_xyz(xy,height).tolist(),
                              'height_m':height,'source':source,'uncertainty_mm':uncertainty_mm,
                              'yaw_table_rad':float(yaw_rad)}
        def correct(xy,height):
            if camera:
                origin=np.asarray(camera['origin_table_m'])
                return origin[:2]+(xy-origin[:2])*(origin[2]-height)/origin[2]
            return xy
        def bag_match(rect,name):
            xy,size,area=rect[:3]
            # Identity comes from appearance and dimensions, not a privileged
            # left-half coordinate. Reject ambiguities instead of selecting a
            # location that happens to match the original demonstration.
            short_max=.34 if name=='bag_black' and name in on_scale else .225
            return .025<area<.10 and .27<np.max(size)<(.36 if name in on_scale else .325) and .175<np.min(size)<short_max
        for name in ('bag_green','bag_black'):
            if name not in required:continue
            matches=[r for r in rectangles(masks[name]) if bag_match(r,name)]
            if not matches:raise VisionError(f'{name} not confidently visible')
            if len(matches)>1:raise VisionError(f'{name} has multiple dimension-consistent candidates; identity is ambiguous')
            chosen=min(matches,key=lambda r:abs(r[2]-.06))
            center=chosen[0].copy()
            corrected_center=None
            if name=='bag_black':
                if name in on_scale and np.min(chosen[1])>.225:
                    if camera is None:raise VisionError('Black parcel merges with dark scale base; reacquire a view with a reliable height estimate')
                    # The dark device and dark parcel form one contour. Its
                    # far and left edges still belong to the larger parcel.
                    # Use image-derived prior yaw and the supplied 300x200 mm
                    # dimensions; never use a commanded or simulated centre.
                    yaw=self.parcel_yaw_priors.get(name,chosen[3])
                    rotation=np.array([[np.cos(yaw),-np.sin(yaw)],[np.sin(yaw),np.cos(yaw)]])
                    points=correct(chosen[4],.032)@rotation
                    width=np.ptp(points[:,0])
                    if not .27<width<.325:raise VisionError('Merged black/scale contour has no parcel-width witness')
                    recovered=rotation@np.array([np.min(points[:,0])+.15,np.max(points[:,1])-.10])
                    put(name,recovered,.032,'RGB parcel overhang edges + known dimensions, dark scale-base merge',15,yaw)
                    continue
                # Bright seal strips are not black. Recover the full known
                # 300 mm width only when a neutral seal is visible on its right.
                long_side=float(np.max(chosen[1]))
                if .27<long_side<.34 and abs(chosen[3])<np.radians(5):
                    # Probe the neutral seal itself, not the thin black lip
                    # beyond it. The former +8 mm probe often missed the seal.
                    u=round((center[0]+long_side/2+.003)*1000)
                    v=round(center[1]*1000)
                    strip=hsv[max(0,v-70):min(600,v+70),max(0,u-4):min(1200,u+4)]
                    if strip.size and np.mean((strip[:,:,1]<45)&(strip[:,:,2]>50))>.3:
                        height=.032 if name in on_scale else .012
                        scale_factor=(camera['origin_table_m'][2]-height)/camera['origin_table_m'][2] if camera else 1
                        physical_width=long_side*scale_factor
                        if .27<physical_width<.30:
                            corrected_center=correct(center,height)
                            corrected_center+=np.array([np.cos(chosen[3]),np.sin(chosen[3])])*(.30-physical_width)/2
                if name not in on_scale and camera and np.max(chosen[4][:,1])>.597 and abs(chosen[3])<np.radians(5):
                    # A far parcel's raised surface can extend past the table
                    # warp. Infer its centre from the visible near edge and
                    # known 200 mm depth, rather than a clipped centroid.
                    if corrected_center is None:corrected_center=correct(center,.012)
                    near_edge=correct(chosen[4],.012)[:,1].min()
                    corrected_center[1]=near_edge+.10
            height=.032 if name in on_scale else .012
            put(name,corrected_center if corrected_center is not None else correct(center,height),height,'RGB appearance in rectified table',10,chosen[3])
        neutral=rectangles(masks['neutral'])
        matches=[r for r in neutral if bag_match(r,'bag_white')]
        if not matches and 'bag_white' in required:
            # Under bright lighting the white cloth can merge into the neutral
            # mask. Tighten saturation until a dimension-consistent white bag separates.
            for threshold in (20,12,7):
                alternatives=rectangles((hsv[:,:,1]<threshold)&(hsv[:,:,2]>150))
                matches=[r for r in alternatives if bag_match(r,'bag_white')]
                if matches:break
        if 'bag_white' in required:
            if not matches:raise VisionError('white bag not confidently visible')
            if len(matches)>1:raise VisionError('white bag has multiple dimension-consistent candidates; identity is ambiguous')
            height=.032 if 'bag_white' in on_scale else .012
            chosen=min(matches,key=lambda r:abs(r[2]-.06));put('bag_white',correct(chosen[0],height),height,'RGB appearance in rectified table',10,chosen[3])
        for name,ident,offset,height in [('scale',10,.14,.02),('material_box',11,.25,.23)]:
            if name not in required:continue
            pair=12 if ident==10 else 13
            if self.allow_object_tags and (ident in tags or pair in tags):
                if pair in tags and ident in tags:
                    left=project(tags[ident],h).mean(0);right=project(tags[pair],h).mean(0)
                    center=(left+right)/2;xdir=right-left;xdir/=np.linalg.norm(xdir)
                    ydir=np.array([-xdir[1],xdir[0]])
                    source='RGB paired base fiducials + known mounting offset'
                else:
                    chosen_id=ident if ident in tags else pair
                    vertices=project(tags[chosen_id],h)
                    center=vertices.mean(0)
                    ydir=(vertices[0]+vertices[1]-vertices[2]-vertices[3])/2
                    ydir/=np.linalg.norm(ydir)
                    xdir=np.array([ydir[1],-ydir[0]])
                    center=center+( .06 if ident==10 else .09)*xdir*(1 if chosen_id==ident else -1)
                    source='RGB single base fiducial + known mounting offset'
                # Suppress a tiny tag-orientation bias for objects aligned with
                # table axes; larger observed rotations remain image-derived.
                if abs(ydir[0])<np.sin(np.radians(2)):
                    ydir=np.array([0.,1. if ydir[1]>0 else -1.])
                put(name,center+offset*ydir,height,source,12,np.arctan2(-ydir[0],ydir[1]))
            else:
                if self.allow_object_tags:
                    raise VisionError(f'{name} base labels are not visible; reacquire a view instead of guessing')
                lo,hi=(.4,.8) if name=='scale' else (.7,1.2)
                expected=.23*.17 if name=='scale' else .4*.3
                matches=[r for r in neutral if lo<r[0][0]<hi and expected*.4<r[2]<expected*3]
                if not matches:raise VisionError(f'{name} not confidently visible')
                chosen=min(matches,key=lambda r:abs(r[2]-expected))
                if not camera and height>.1:raise VisionError('Raised box without base tag requires a reliable camera-height estimate from RGB')
                put(name,correct(chosen[0],height),height,'RGB appearance + inferred height correction',20 if name=='material_box' else 10)
        return {'detections':detections,'table_homography_pixel_to_m':h.tolist(),
                'quality':quality,'camera_self_calibration':camera,'manual_calibration_residual_mm':self.calibration.max_residual_mm}

    def detect_sequence(self, frames, minimum=3, agreement_m=.02, required=None,on_scale=None):
        """Require image-derived agreement, without any known object positions."""
        predictions=[];rejections=[];frame_indices=[]
        for frame_index,rgb in enumerate(frames):
            try:
                predictions.append(self.detect(rgb,required=required,on_scale=on_scale));frame_indices.append(frame_index)
            except VisionError as error:rejections.append(str(error))
        if len(predictions)<minimum:raise VisionError(f'Only {len(predictions)} trustworthy frames; need {minimum}. '+str(rejections))
        names=sorted(predictions[0]['detections'])
        vectors=np.array([[p['detections'][n]['table_xy_m'] for n in names] for p in predictions])
        distances=np.linalg.norm(vectors[:,None]-vectors[None,:],axis=-1).max(axis=-1)
        best=np.argmax((distances<agreement_m).sum(axis=1))
        indices=np.flatnonzero(distances[best]<agreement_m)
        if len(indices)<minimum:raise VisionError('RGB observations disagree; do not grasp')
        median=np.median(vectors[indices],axis=0)
        result=dict(predictions[int(indices[0])]);result['detections']={}
        for i,name in enumerate(names):
            original=predictions[int(indices[0])]['detections'][name]
            spread=np.linalg.norm(vectors[indices,i]-median[i],axis=1)
            result['detections'][name]={**original,'table_xy_m':median[i].tolist(),
                'robot_xyz_m':self.calibration.robot_xyz(median[i],original['height_m']).tolist(),
                'source':'RGB temporal consensus: '+original['source'],
                'uncertainty_mm':max(float(original['uncertainty_mm']),float(np.percentile(spread,90)*1000))}
        result['consensus']={'frames_received':len(predictions)+len(rejections),'valid_frames':len(predictions),
                             'consistent_frames':len(indices),'rejected_frames':len(rejections),
                             'accepted_frame_indices':[frame_indices[int(i)] for i in indices]}
        for name,det in result['detections'].items():
            if name.startswith('bag_') and name not in set(on_scale or []):self.parcel_yaw_priors[name]=det.get('yaw_table_rad',0)
        return result
