#!/usr/bin/env python3
"""RGB evidence of clearing a source region; no simulator imports/poses.

Source clearance alone cannot prove a physical grasp. A subsequent observed
placement and nonzero scale reading are still required.
"""
import cv2,numpy as np
from kuavo_table_vision import VisionError


def rectified(vision,rgb):
    h,tags,quality=vision.table_homography(rgb);references=[]
    for vertices in tags.values():
        mask=np.zeros(rgb.shape[:2],np.uint8);cv2.fillConvexPoly(mask,vertices.round().astype(np.int32),1)
        pixels=rgb[mask>0]
        if len(pixels)>20:references.append(np.percentile(pixels,90,axis=0))
    if references:rgb=np.clip(rgb.astype(float)*255/np.maximum(np.median(references,axis=0),30),0,255).astype(np.uint8)
    transform=np.diag([1000,1000,1])@h
    warp=cv2.warpPerspective(rgb,transform,(1200,600))
    visible=cv2.warpPerspective(np.ones(rgb.shape[:2],np.uint8),transform,(1200,600),flags=cv2.INTER_NEAREST)>0
    return cv2.cvtColor(warp,cv2.COLOR_RGB2LAB).astype(float),visible


def region(xy,yaw,width,height):
    rotation=np.array([[np.cos(yaw),-np.sin(yaw)],[np.sin(yaw),np.cos(yaw)]])
    points=np.array([[-width/2,-height/2],[width/2,-height/2],[width/2,height/2],[-width/2,height/2]])@rotation.T+xy
    mask=np.zeros((600,1200),np.uint8);cv2.fillConvexPoly(mask,(points*1000).round().astype(np.int32),1)
    return mask>0


class SourceClearanceVerifier:
    def __init__(self,vision,reference_rgb,detection,other_detections=None):
        self.vision=vision;xy=np.asarray(detection['table_xy_m']);yaw=detection.get('yaw_table_rad',0)
        lab,visible=rectified(vision,reference_rgb)
        self.roi=region(xy,yaw,.26,.16)
        outer=region(xy,yaw,.40,.30);inner=region(xy,yaw,.32,.22)
        ring=outer&~inner&visible
        for other in (other_detections or {}).values():
            if other is detection:continue
            center=np.asarray(other['table_xy_m'])
            if np.linalg.norm(center-xy)<.02:continue
            # Exclude all known visible parcels/devices from the background
            # teaching ring; a homogeneous green neighbour otherwise wins the
            # histogram over cloth whose luminance varies under arm shadows.
            ring&=~region(center,other.get('yaw_table_rad',0),.42 if other.get('height_m',0)>.1 else .32,.32 if other.get('height_m',0)>.1 else .22)
        for x,y in vision.tag_centers.values():ring[max(0,round(y*1000)-40):min(600,round(y*1000)+40),max(0,round(x*1000)-40):min(1200,round(x*1000)+40)]=False
        pixels=lab[ring]
        if len(pixels)<1000 or np.mean(visible[self.roi])<.9:raise VisionError('Insufficient visible source/background for visual verification')
        bins,counts=np.unique((pixels[:,1:]/8).astype(int),axis=0,return_counts=True);mode=bins[np.argmax(counts)]
        selected=pixels[np.all((pixels[:,1:]/8).astype(int)==mode,axis=1)]
        self.background=np.median(selected,axis=0);self.parcel=np.median(lab[self.roi&visible],axis=0)
        self.contrast=float(np.linalg.norm(self.background-self.parcel))
        self.chroma_contrast=float(np.linalg.norm(self.background[1:]-self.parcel[1:]))
        if self.contrast<15:raise VisionError('Source and table background are too similar to verify source clearance')

    def score(self,rgb):
        lab,visible=rectified(self.vision,rgb);visibility=float(np.mean(visible[self.roi]))
        if visibility<.9:raise VisionError('Source region is outside the current RGB frame')
        pixels=lab[self.roi&visible];bg=np.linalg.norm(pixels-self.background,axis=1);bag=np.linalg.norm(pixels-self.parcel,axis=1)
        matched=(bg<45)&(bg+8<bag)
        # Arm shadows change brightness much more than cloth chromaticity.
        # Only add this witness when the taught parcel/cloth chroma is clearly
        # separated AND the full LAB colour still favours the table.
        if self.chroma_contrast>=16:
            cbg=np.linalg.norm(pixels[:,1:]-self.background[1:],axis=1)
            cbag=np.linalg.norm(pixels[:,1:]-self.parcel[1:],axis=1)
            matched|=(cbg<12)&(cbg+8<cbag)&(bg+8<bag)
        fraction=float(np.mean(matched))
        return {'source_visible_fraction':visibility,'visible_table_background_fraction':fraction,
                'source_cleared':fraction>=.60,'reference_color_contrast_lab':self.contrast,
                'reference_chroma_contrast_lab':self.chroma_contrast}

    def verify_sequence(self,frames,minimum=3):
        rows=[];errors=[]
        for rgb in frames:
            try:rows.append(self.score(rgb))
            except VisionError as error:errors.append(str(error))
        accepted=sum(row['source_cleared'] for row in rows)
        report={'verified':accepted>=minimum,'agreeing_frames':accepted,'valid_frames':len(rows),'frames':rows,'errors':errors,
                'meaning':'visible table background replaces the source parcel after transport; this is not proof of finger contact or holding force'}
        return report
