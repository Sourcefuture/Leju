"""Pure RGB disturbances for simulation experiments; no geometry or simulator API."""
import cv2,numpy as np

def perturb_rgb(rgb,rng,level):
    if level==0:return rgb
    gain=rng.uniform(.75,1.25);gamma=rng.uniform(.8,1.3)
    image=np.clip(((rgb/255.)**gamma)*255*gain,0,255)
    if level>=2:
        image*=rng.uniform(.85,1.15,size=(1,1,3))
        image+=rng.normal(0,rng.uniform(1,7),size=image.shape)
    image=np.clip(image,0,255).astype(np.uint8)
    if rng.random()<.35:image=cv2.GaussianBlur(image,(3,3),rng.uniform(.3,.8))
    if level>=2:
        _,encoded=cv2.imencode('.jpg',cv2.cvtColor(image,cv2.COLOR_RGB2BGR),[cv2.IMWRITE_JPEG_QUALITY,int(rng.integers(45,96))])
        image=cv2.cvtColor(cv2.imdecode(encoded,cv2.IMREAD_COLOR),cv2.COLOR_BGR2RGB)
    return image


def apply_conditions(rgb,rng,conditions):
    """Camera-like image disturbances; only pixels and declared settings enter."""
    c=conditions or {};image=rgb.copy();h,w=image.shape[:2]
    k=float(c.get('radial_k1',0))
    if not np.isfinite(k) or abs(k)>.5:raise ValueError('radial_k1 must be finite and within +/-0.5')
    if k:
        u,v=np.meshgrid(np.arange(w,dtype=np.float32),np.arange(h,dtype=np.float32))
        x=(u-w/2)/w;y=(v-h/2)/w;s=1+k*(x*x+y*y)
        image=cv2.remap(image,w/2+x*w*s,h/2+y*h*s,cv2.INTER_LINEAR,borderMode=cv2.BORDER_REFLECT)
    exposure=float(c.get('exposure_gain',1))
    if not np.isfinite(exposure) or not .15<=exposure<=2:raise ValueError('exposure_gain must be .15..2')
    vignette=float(c.get('vignette',0))
    if not np.isfinite(vignette) or not 0<=vignette<=.9:raise ValueError('vignette must be 0..0.9')
    if exposure!=1 or vignette:
        u,v=np.meshgrid(np.linspace(-1,1,w),np.linspace(-1,1,h))
        gain=exposure*(1-vignette*np.clip((u*u+v*v)/2,0,1))
        image=np.clip(image.astype(float)*gain[:,:,None],0,255).astype(np.uint8)
    for rect in c.get('occlusions',[]):
        if len(rect)!=4 or not np.isfinite(rect).all():raise ValueError('Occlusion needs normalized x,y,w,h')
        x,y,rw,rh=rect
        if min(x,y,rw,rh)<0 or x+rw>1 or y+rh>1:raise ValueError('Occlusion must lie inside RGB image')
        image[round(y*h):round((y+rh)*h),round(x*w):round((x+rw)*w)]=c.get('occlusion_color_rgb',[100,100,100])
    fraction=float(c.get('random_occlusion_fraction',0))
    if not np.isfinite(fraction) or not 0<=fraction<=.3:raise ValueError('Random occlusion fraction must be 0..0.3')
    if fraction:
        edge=round(np.sqrt(h*w*fraction));edge=min(edge,h,w)
        x=int(rng.integers(0,w-edge+1));y=int(rng.integers(0,h-edge+1))
        image[y:y+edge,x:x+edge]=rng.integers(30,220,3,dtype=np.uint8)
    return image

