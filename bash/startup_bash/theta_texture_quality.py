"""General floor-texture sampling and quality/consistency fusion.

No semantic class, marking detector, line straightening, or coordinate warp is
used. Native rasterization and inverse cell-center sampling share the existing
camera model and recorded poses. Scan visibility uses the production gate.
"""
import math

import cv2
import numpy as np


def camera_rotation(calibration):
    roll,pitch,yaw=np.deg2rad(calibration['rpy_degrees']);pitch=-pitch
    cr,sr,cp,sp,cy,sy=np.cos(roll),np.sin(roll),np.cos(pitch),np.sin(pitch),np.cos(yaw),np.sin(yaw)
    return np.array([[cy,-sy,0],[sy,cy,0],[0,0,1]])@np.array([[cp,0,sp],[0,1,0],[-sp,0,cp]])@np.array([[1,0,0],[0,cr,-sr],[0,sr,cr]])


def sampling_maps(calibration,x,y,image_shape,blend_half_deg=6.):
    """Inverse-map arbitrary robot-floor coordinates to calibrated fisheyes."""
    from sirius_navigation.theta_bev_projection import _lens_uv
    ground=np.stack((x,y,np.zeros_like(x)),axis=-1)
    rays=(ground-np.asarray(calibration['camera_position']))@camera_rotation(calibration)
    rays/=np.linalg.norm(rays,axis=-1,keepdims=True)
    scale=np.array([image_shape[1],image_shape[0]])/np.asarray(calibration['image_size'])
    front,valid_f=_lens_uv(calibration['front'],1,rays,scale)
    back,valid_b=_lens_uv(calibration['back'],-1,rays,scale)
    if blend_half_deg>0:
        s=math.sin(math.radians(blend_half_deg));alpha=np.clip((rays[...,0]+s)/(2*s),0,1)
    else:alpha=(rays[...,0]>=0).astype(float)
    alpha=np.where(valid_f,alpha,0);alpha=np.where(valid_b,alpha,1);alpha=np.where(~valid_f&~valid_b,0,alpha)
    return tuple(np.asarray(v,np.float32) for v in (
        np.where(valid_f,front[...,0],-1),np.where(valid_f,front[...,1],-1),
        np.where(valid_b,back[...,0],-1),np.where(valid_b,back[...,1],-1),alpha,
        .15+.85*rays[...,0]**2))


def sample_floor(raw,calibration,x,y):
    fx,fy,bx,by,alpha,lens=sampling_maps(calibration,x,y,raw.shape)
    front=cv2.remap(raw,fx,fy,cv2.INTER_LINEAR,borderMode=cv2.BORDER_CONSTANT)
    back=cv2.remap(raw,bx,by,cv2.INTER_LINEAR,borderMode=cv2.BORDER_CONSTANT)
    rgb=np.clip(front.astype(np.float32)*alpha[...,None]+back.astype(np.float32)*(1-alpha[...,None]),0,255).astype(np.uint8)
    return rgb,lens


def aggregate_frame(index,bgr,quality,lens,rear):
    """One color per frame/cell; preserve summed weights for baseline parity."""
    unique,inverse=np.unique(index,return_inverse=True)
    weight=np.bincount(inverse,weights=quality)
    color=np.column_stack([np.bincount(inverse,weights=quality*bgr[:,c]) for c in range(3)])/np.maximum(weight[:,None],1e-9)
    q=np.zeros(len(unique),np.float32);np.maximum.at(q,inverse,quality)
    l=np.bincount(inverse,weights=quality*lens)/np.maximum(weight,1e-9)
    r=np.bincount(inverse,weights=quality*rear)/np.maximum(weight,1e-9)
    return dict(index=unique.astype(np.int32),bgr=color.astype(np.float32),q=q,
                weight=weight.astype(np.float32),lens=l.astype(np.float32),rear=r.astype(np.float32))


class TextureFusion:
    """Cell-local fusion; changes neither coordinates nor observation support."""
    def __init__(self,size,mode='ema',relative_quality=.0,color_tolerance=25.,decay=.7):
        self.mode=mode;self.relative=relative_quality;self.tolerance=color_tolerance;self.decay=decay
        self.sum=np.zeros((size,3),np.float32);self.weight=np.zeros(size,np.float32)
        self.best=np.zeros(size,np.float32);self.anchor=np.zeros((size,3),np.float32)
        self.accepted=0;self.rejected_color=0

    def update(self,index,bgr,quality,weight=None):
        if len(index)==0:return
        index=np.asarray(index);bgr=np.asarray(bgr,np.float32);quality=np.asarray(quality,np.float32)
        valid=np.isfinite(quality)&(quality>0)&np.isfinite(bgr).all(axis=1)
        index,bgr,quality=index[valid],bgr[valid],quality[valid]
        weights=quality.copy() if weight is None else np.asarray(weight,np.float32)[valid]
        better=quality>self.best[index];unseen=self.weight[index]<=0
        if self.mode=='best':
            keep=better|unseen
            i=index[keep];self.sum[i]=bgr[keep]*quality[keep,None];self.weight[i]=quality[keep]
            self.best[index]=np.maximum(self.best[index],quality);self.accepted+=int(keep.sum());return
        keep=quality>=self.relative*np.maximum(self.best[index],quality)
        if self.mode=='robust':
            consistent=np.mean(np.abs(bgr-self.anchor[index]),axis=1)<=self.tolerance
            reset=(better|unseen)&~consistent
            self.sum[index[reset]]=0;self.weight[index[reset]]=0
            self.anchor[index[better|unseen]]=bgr[better|unseen]
            accept_color=consistent|better|unseen
            self.rejected_color+=int((keep&~accept_color).sum());keep&=accept_color
            # Per-frame quality, rather than sample density, controls blending.
            weights=quality
        i=index[keep];w=weights[keep]
        self.sum[i]=self.decay*self.sum[i]+w[:,None]*bgr[keep]
        self.weight[i]=self.decay*self.weight[i]+w
        self.best[index]=np.maximum(self.best[index],quality);self.accepted+=int(keep.sum())

    def result(self):
        return np.clip(np.rint(self.sum/np.maximum(self.weight[:,None],1e-9)),0,255).astype(np.uint8),self.weight>0


VARIANTS={
    'scatter_ema':dict(source='native',mode='ema',relative=0,rear=False,lens=False),
    'scatter_protected':dict(source='native',mode='ema',relative=.5,rear=True,lens=False),
    'scatter_protected_lens':dict(source='native',mode='ema',relative=.5,rear=True,lens=True),
    'direct_ema':dict(source='direct',mode='ema',relative=0,rear=False,lens=False),
    'direct_protected_no_lens':dict(source='direct',mode='ema',relative=.5,rear=True,lens=False),
    'direct_protected':dict(source='direct',mode='ema',relative=.5,rear=True,lens=True),
    'direct_best':dict(source='direct',mode='best',relative=0,rear=True,lens=True),
    'direct_robust065_25':dict(source='direct',mode='robust',relative=.65,rear=True,lens=True,tolerance=25),
    'direct_robust080_25':dict(source='direct',mode='robust',relative=.8,rear=True,lens=True,tolerance=25),
    'direct_robust065_40':dict(source='direct',mode='robust',relative=.65,rear=True,lens=True,tolerance=40),
}


def feed_variant(fuser,packet,variant,index_map=None):
    p=packet[variant['source']];q=p['q'].copy();w=p['weight'].copy()
    if variant['rear']:q*=p['rear'];w*=p['rear']
    if variant['lens']:q*=p['lens'];w*=p['lens']
    idx=p['index'] if index_map is None else index_map(p['index'])
    valid=idx>=0
    fuser.update(idx[valid],p['bgr'][valid],q[valid],w[valid])
