#!/usr/bin/env python3
# Vertical (N-S elongated) topology PREDICTION (synthesis, no SITL).
# Phase order: takeoff -> azimuth(125->30 CCW) -> 1/4 climb -> climbX -> accel(cruise72)
#   -> turn(CCW) -> brake -> descentX -> turn(CCW) -> 1/4 descend -> land
import numpy as np, matplotlib
matplotlib.use("Agg"); import matplotlib.pyplot as plt
out="/tmp/mk7_vert_predict.png"
R=72.0**2/(9.81*np.tan(np.radians(45)))    # 528 m turn radius @72/45deg
CLIMB=3119.0; DESC_P9=3613.0; DESC_14=950.0 # measured footprints
ACCEL=500.0; BRAKE=500.0                      # nominal cruise-accel & brake horizontal
def cruise(p,hdg,L,n=24):
    h=np.radians(hdg); return np.c_[p[0]+np.linspace(0,L,n)*np.sin(h), p[1]+np.linspace(0,L,n)*np.cos(h)]
def arc(p,hdg0,turn,R,n=60):      # turn>0=CW(right), turn<0=CCW(left)
    h0=np.radians(hdg0); s=np.sign(turn); cx=p[0]+s*R*np.cos(h0); cy=p[1]-s*R*np.sin(h0)
    a0=np.arctan2(p[1]-cy,p[0]-cx); aa=a0 - s*np.radians(abs(turn))*np.linspace(0,1,n)
    return np.c_[cx+R*np.cos(aa), cy+R*np.sin(aa)]
HOME=np.array([0.0,0.0]); segs=[]; p=HOME.copy(); h=30.0   # after azimuth 125->30
def add(name,s,col,seth=None):
    global p,h
    segs.append((name,s,col)); p=s[-1]
    if seth is not None: h=seth
    else:
        d=s[-1]-s[-2]; h=np.degrees(np.arctan2(d[0],d[1]))%360
add("climb 1/4+X (30deg, 1:4->152->300m)", cruise(p,h,CLIMB), "#2ca02c")
add("accelerate (cruise72)", cruise(p,h,ACCEL), "#17becf")
add("turn 1 CCW 180", arc(p,h,-180,R), "#d62728")
add("brake (P8)", cruise(p,h,BRAKE), "#ff7f0e")
add("descent X (210deg, P9 300->152 38m/s)", cruise(p,h,DESC_P9), "#9467bd")
add("turn 2 CCW 180", arc(p,h,-180,R), "#e377c2")
add("descend 1/4 (152->10m)", cruise(p,h,DESC_14), "#8c564b")
land=p.copy()
allpts=np.vstack([s for _,s,_ in segs])
ext_E=allpts[:,0].max()-allpts[:,0].min(); ext_N=allpts[:,1].max()-allpts[:,1].min()
closure=np.hypot(*land)
fig,ax=plt.subplots(figsize=(8.5,11))
for nm,s,c in segs: ax.plot(s[:,0],s[:,1],color=c,lw=2.6,label=nm)
ax.plot(0,0,"*",color="gold",ms=26,mec="k",zorder=6,label="HOME (takeoff)")
ax.plot(land[0],land[1],"X",color="red",ms=15,mec="k",zorder=6,label="LAND (predicted)")
ax.annotate("",xy=cruise(HOME,30,400)[-1],xytext=(0,0),arrowprops=dict(arrowstyle="->",lw=2,color="k"))
ax.set_aspect("equal"); ax.grid(alpha=.3); ax.set_xlabel("East [m]"); ax.set_ylabel("North [m]")
ax.set_title("Mk-7 VERTICAL (N-S) topology PREDICTION (synthesis)\n"
  "out 30deg NNE, 2x CCW 180; extent N-S=%.1fkm E-W=%.1fkm; LAND offset from HOME=%.0f m"%(ext_N/1000,ext_E/1000,closure),fontsize=10)
ax.legend(loc="upper right",fontsize=7.5)
plt.tight_layout(); plt.savefig(out,dpi=125)
print("SAVED",out)
print("extent N-S=%.0f m  E-W=%.0f m  -> %s"%(ext_N,ext_E,"VERTICAL(N-S long)" if ext_N>ext_E else "WIDE(E-W long)"))
print("LAND predicted=(%.0f,%.0f)  closure(dist HOME)=%.0f m"%(land[0],land[1],closure))
