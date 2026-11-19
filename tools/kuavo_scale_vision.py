#!/usr/bin/env python3
"""Seven-segment scale display generation and RGB-only reading for simulation."""
from __future__ import annotations
import cv2
import numpy as np

SEGMENTS={
    '0':(1,1,1,1,1,1,0),'1':(0,1,1,0,0,0,0),'2':(1,1,0,1,1,0,1),
    '3':(1,1,1,1,0,0,1),'4':(0,1,1,0,0,1,1),'5':(1,0,1,1,0,1,1),
    '6':(1,0,1,1,1,1,1),'7':(1,1,1,0,0,0,0),'8':(1,1,1,1,1,1,1),'9':(1,1,1,1,0,1,1),
}
WIDTH,HEIGHT=600,160
BOXES=[(18,8,72,23),(65,18,82,75),(65,85,82,142),(18,137,72,152),
       (8,85,25,142),(8,18,25,75),(18,72,72,88)]
LCD_THEMES={'green':([100,205,165],[12,35,27]),'gray':([190,195,180],[25,30,25]),
            'blue':([40,100,165],[225,235,245]),'white':([235,233,225],[20,25,20])}


def rectify_lcd_crop(crop):
    hsv=cv2.cvtColor(crop,cv2.COLOR_RGB2HSV)
    masks=[(hsv[:,:,0]>35)&(hsv[:,:,0]<100)&(hsv[:,:,1]>50)&(hsv[:,:,2]>40),
           (hsv[:,:,0]>=95)&(hsv[:,:,0]<135)&(hsv[:,:,1]>50)&(hsv[:,:,2]>40),
           (hsv[:,:,1]<45)&(hsv[:,:,2]>90)]
    candidates=[]
    for raw in masks:
        mask=cv2.morphologyEx(raw.astype(np.uint8)*255,cv2.MORPH_CLOSE,np.ones((3,3),np.uint8))
        contours,_=cv2.findContours(mask,cv2.RETR_EXTERNAL,cv2.CHAIN_APPROX_SIMPLE)
        for contour in contours:
            dims=cv2.minAreaRect(contour)[1];short=min(dims);long=max(dims)
            if short>12 and 3.3<long/short<7.5 and cv2.contourArea(contour)>600:candidates.append(contour)
    if not candidates:raise ValueError('Scale LCD background rectangle not visible')
    contour=max(candidates,key=cv2.contourArea);hull=cv2.convexHull(contour)
    polygon=cv2.approxPolyDP(hull,.012*cv2.arcLength(hull,True),True)
    points=polygon[:,0,:] if len(polygon)==4 else cv2.boxPoints(cv2.minAreaRect(hull))
    sums=points.sum(1);diff=points[:,0]-points[:,1]
    ordered=points[[np.argmin(sums),np.argmax(diff),np.argmax(sums),np.argmin(diff)]]
    if len(set(map(tuple,ordered)))!=4:raise ValueError('LCD corners are ambiguous')
    mapping=cv2.getPerspectiveTransform(ordered.astype(np.float32),np.array([[0,0],[WIDTH-1,0],[WIDTH-1,HEIGHT-1],[0,HEIGHT-1]],np.float32))
    return np.flipud(cv2.warpPerspective(crop,mapping,(WIDTH,HEIGHT))).copy()


def make_lcd(mass_kg,theme='green'):
    tenths=int(round(mass_kg*10000))
    if not 0<=tenths<=50000:raise ValueError('Scale load outside 0..5 kg')
    digits=f'{tenths:05d}'
    if theme not in LCD_THEMES:raise ValueError('Unknown LCD theme')
    background,ink=LCD_THEMES[theme]
    image=np.zeros((HEIGHT,WIDTH,3),np.uint8);image[:]=background
    for i,digit in enumerate(digits):
        x0=35+i*105
        for active,(x1,y1,x2,y2) in zip(SEGMENTS[digit],BOXES):
            if active:cv2.rectangle(image,(x0+x1,y1),(x0+x2,y2),ink,-1)
    cv2.circle(image,(35+4*105-8,145),7,ink,-1)
    return image


def texture_theme(model):
    ident=model.texture('scale_lcd_texture').id;adr=int(model.tex_adr[ident]);color=model.tex_data[adr:adr+3].astype(float)
    return min(LCD_THEMES,key=lambda key:np.linalg.norm(color-LCD_THEMES[key][0]))


def write_lcd_texture(model, mass_kg):
    """Device/replay helper; the decoder never accesses this model."""
    try:ident=model.texture('scale_lcd_texture').id
    except KeyError:return None
    image=make_lcd(mass_kg,texture_theme(model))
    adr=int(model.tex_adr[ident]);model.tex_data[adr:adr+image.size]=image.ravel()
    return ident


def decode_lcd(rgb):
    """Recognize the printed font with per-digit illumination and registration."""
    resized=cv2.resize(rgb,(WIDTH,HEIGHT),interpolation=cv2.INTER_CUBIC)
    resized=cv2.GaussianBlur(resized,(3,3),.6)
    gray=cv2.cvtColor(resized,cv2.COLOR_RGB2GRAY)
    color_ratio=resized[:,:,0].astype(float)/np.maximum(resized[:,:,1],1)
    saturation=cv2.cvtColor(resized,cv2.COLOR_RGB2HSV)[:,:,1]
    patterns={}
    for digit,segments in SEGMENTS.items():
        template=np.zeros((160,100),np.uint8)
        for active,(x1,y1,x2,y2) in zip(segments,BOXES):
            if active:cv2.rectangle(template,(x1,y1),(x2,y2),1,-1)
        patterns[digit]=template[8:153,8:83].astype(np.float32)
    digits=[];errors=[];margins=[];polarity=None
    for index in range(5):
        patch=gray[:,max(0,35+index*105-8):min(WIDTH,35+index*105+98)]
        low,high=np.percentile(patch,[5,90])
        if high-low<20:raise ValueError('LCD contrast too low; reacquire a view')
        binary=(patch<(low+high)/2).astype(np.float32)
        def classify(binary):
            candidates=[]
            for digit,template in patterns.items():
                best=1.
                for scale in [.90,.95,1.,1.05]:
                    fitted=cv2.resize(template,None,fx=scale,fy=scale,interpolation=cv2.INTER_LINEAR)
                    scores=cv2.matchTemplate(binary,fitted,cv2.TM_SQDIFF)
                    value,_,_,_=cv2.minMaxLoc(scores);best=min(best,value/fitted.size)
                candidates.append((best,digit))
            candidates.sort();return candidates
        candidates=classify(binary)
        inverted=classify(1-binary)
        if polarity is None:polarity='light_ink' if inverted[0][0]<candidates[0][0] else 'dark_ink'
        if polarity=='light_ink':candidates=inverted
        if candidates[0][0]>.16 or candidates[1][0]-candidates[0][0]<.025:
            # Green LCD ink and background have different chromatic ratios;
            # this recovers a digit crossed by an irregular illumination edge.
            if polarity=='dark_ink' and np.mean(cv2.cvtColor(resized,cv2.COLOR_RGB2HSV)[:,:,1]>50)>.25:
                patch=color_ratio[:,max(0,35+index*105-8):min(WIDTH,35+index*105+98)]
                candidates=classify((patch<np.percentile(patch,85)*.85).astype(np.float32))
            elif polarity=='light_ink':
                # Neutral light ink on a saturated blue backlight remains
                # separable when perspective/mipmapping blurs luminance edges.
                patch=saturation[:,max(0,35+index*105-8):min(WIDTH,35+index*105+98)]
                low_s,high_s=np.percentile(patch,[5,90])
                if high_s-low_s>45:candidates=classify((patch<(low_s+high_s)/2).astype(np.float32))
        error,digit=candidates[0];margin=candidates[1][0]-error
        if error>.16 or margin<.025:raise ValueError('LCD glyph is ambiguous; reacquire a clearer view')
        digits.append(digit);errors.append(error);margins.append(margin)
    grams=int(''.join(digits))/10
    if grams>5000:raise ValueError('Scale display exceeds rated range')
    return {'measured_kg':grams/1000,'display_grams':grams,'digits':''.join(digits),
            'segment_confidence':float(min(margins)),'max_glyph_fit_error':float(max(errors)),
            'source':'RGB registered seven-segment display only'}


def decode_lcd_fixed_grid(rgb):
    resized=cv2.resize(rgb,(WIDTH,HEIGHT),interpolation=cv2.INTER_CUBIC)
    gray=cv2.cvtColor(resized,cv2.COLOR_RGB2GRAY).astype(float)
    bits=[];digits=[];confidence=[]
    for i in range(5):
        # A robot arm may shadow only part of the LCD. A global threshold can
        # turn a shadowed digit into 8; estimate illumination separately per cell.
        cell=gray[:,35+i*105:35+(i+1)*105]
        low,high=np.percentile(cell,[10,90])
        if high-low<20:raise ValueError('LCD contrast too low; reacquire a view')
        threshold=(low+high)/2
        samples=[]
        for x1,y1,x2,y2 in BOXES:
            # Sample segment cores, not anti-aliased edges.
            dx=max(2,(x2-x1)//4);dy=max(2,(y2-y1)//4)
            patch=gray[y1+dy:y2-dy,35+i*105+x1+dx:35+i*105+x2-dx]
            samples.append(float(np.mean(patch<threshold)))
        vector=np.array(samples)>.45
        distances={d:int(np.count_nonzero(vector!=np.array(pattern))) for d,pattern in SEGMENTS.items()}
        digit=min(distances,key=distances.get)
        if distances[digit]>0:raise ValueError('Ambiguous scale segments; acquire a clearer image')
        bits.append(vector.astype(int).tolist());digits.append(digit)
        confidence.append(float(min(abs(np.asarray(samples)-.45))))
    grams=int(''.join(digits))/10
    if grams>5000:raise ValueError('Scale display exceeds rated range')
    return {'measured_kg':grams/1000,'display_grams':grams,'digits':''.join(digits),
            'segment_confidence':min(confidence),'source':'RGB seven-segment display only'}


def read_scale_sequence(frames,minimum=3):
    """Accept repeated agreeing RGB readings; no expected weight is supplied."""
    readings=[];crops=[];source_indices=[]
    for source_index,rgb in enumerate(frames):
        try:
            reading,crop=read_scale_local_rgb(rgb)
            readings.append(reading);crops.append(crop);source_indices.append(source_index)
        except ValueError:continue
    groups={}
    for index,reading in enumerate(readings):groups.setdefault(reading['digits'],[]).append(index)
    if not groups:raise ValueError('No trustworthy scale RGB reading')
    chosen=max(groups.values(),key=len)
    if len(chosen)<minimum:raise ValueError('Scale RGB readings do not yet agree')
    if sum(len(indices)>=minimum for indices in groups.values())>1:
        raise ValueError('Conflicting agreeing RGB scale readings; reacquire before continuing')
    index=max(chosen,key=lambda i:readings[i]['segment_confidence'])
    return {**readings[index],'agreeing_frames':len(chosen),'valid_frames':len(readings),
            'accepted_frame_indices':[source_indices[i] for i in chosen],
            'selected_frame_index':source_indices[index]},crops[index]


def read_scale_rgb(rgb, observation):
    h=np.asarray(observation['table_homography_pixel_to_m'])
    scale=observation['detections']['scale']['table_xy_m']
    warped=cv2.warpPerspective(rgb,np.diag([2000,2000,1])@h,(2400,1200))
    x,y=scale;cx,cy=int(x*2000),int((y-.0675)*2000)
    x1,x2=max(0,cx-215),min(2400,cx+215);y1,y2=max(0,cy-75),min(1200,cy+75)
    crop=warped[y1:y2,x1:x2]
    lcd=rectify_lcd_crop(crop)
    return decode_lcd(lcd),lcd


def rectify_scale_local_rgb(rgb):
    """Rectify the LCD from its local labels; no table corners are required."""
    params=cv2.aruco.DetectorParameters()
    params.cornerRefinementMethod=cv2.aruco.CORNER_REFINE_SUBPIX;params.cornerRefinementWinSize=3
    detector=cv2.aruco.ArucoDetector(cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50),params)
    corners,ids,_=detector.detectMarkers(cv2.cvtColor(rgb,cv2.COLOR_RGB2GRAY))
    tags={} if ids is None else {int(i):p.reshape(4,2) for i,p in zip(ids[:,0],corners)}
    image=[];local=[]
    for ident,x in [(10,-.06),(12,.06)]:
        if ident not in tags:continue
        image.extend(tags[ident]);local.extend([(x-.03,-.11),(x+.03,-.11),(x+.03,-.17),(x-.03,-.17)])
    if len(image)<4:raise ValueError('Scale base label not visible for LCD rectification')
    h,_=cv2.findHomography(np.asarray(image),np.asarray(local),0)
    mapping=np.array([[2000,0,300],[0,2000,400],[0,0,1]])@h
    warped=cv2.warpPerspective(rgb,mapping,(600,700))
    crop=warped[190:340,85:515]
    lcd=rectify_lcd_crop(crop)
    return lcd


def read_scale_local_rgb(rgb):
    lcd=rectify_scale_local_rgb(rgb)
    return decode_lcd(lcd),lcd
