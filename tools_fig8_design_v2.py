#!/usr/bin/env python3
# Figure-8 design v2 (distance-budget): climb leg (45 deg) -> COMPACT infinity at climb-top -> descent leg -> HOME.
import numpy as np, matplotlib
matplotlib.use("Agg"); import matplotlib.pyplot as plt
out = "/tmp/mk7_fig8_design_v2.png"
R = 72.0**2/(9.81*np.tan(np.radians(45)))      # 528 m
CLIMB = 3119.0; DESC = 4559.0                  # measured footprints
def cruise(p, hdg, L, n=30):
    h = np.radians(hdg); return np.c_[p[0]+np.linspace(0,L,n)*np.sin(h), p[1]+np.linspace(0,L,n)*np.cos(h)]
def arc(p, hdg0, turn, R, n=70):
    h0 = np.radians(hdg0); s = np.sign(turn); cx = p[0]+s*R*np.cos(h0); cy = p[1]-s*R*np.sin(h0)
    a0 = np.arctan2(p[1]-cy, p[0]-cx); aa = a0 - s*np.radians(abs(turn))*np.linspace(0,1,n)
    return np.c_[cx+R*np.cos(aa), cy+R*np.sin(aa)]
HOME = np.array([0.0,0.0])
# climb leg 45 deg to the climb-top A
A = HOME + CLIMB*np.array([np.sin(np.radians(45)), np.cos(np.radians(45))])
climb = cruise(HOME, 45, CLIMB)
# COMPACT infinity starting at A heading 45 (the climb already established 45 -> clean entry)
segs = []; p = A.copy(); h = 45.0
s = cruise(p,h,R); segs.append(("inf leg1 (out)",s)); p = s[-1]
s = arc(p,h,270,R); segs.append(("Turn A CW270 (East lobe)",s)); p = s[-1]; d = s[-1]-s[-2]; h = np.degrees(np.arctan2(d[0],d[1]))%360
s = cruise(p,h,2*R); segs.append(("cross (through inf-centre)",s)); p = s[-1]
s = arc(p,h,-270,R); segs.append(("Turn B CCW270 (West lobe)",s)); p = s[-1]; d = s[-1]-s[-2]; h = np.degrees(np.arctan2(d[0],d[1]))%360
s = cruise(p,h,R); segs.append(("inf leg3 -> close",s)); p = s[-1]
inf_close = p.copy()
selfcl = np.hypot(*(inf_close-A))
# descent leg: from the infinity-close point back to HOME (the existing return machinery), schematic curve
dn = 40; tt = np.linspace(0,1,dn)
desc = np.c_[inf_close[0]*(1-tt) + HOME[0]*tt + 300*np.sin(np.pi*tt), inf_close[1]*(1-tt) + HOME[1]*tt - 300*np.sin(np.pi*tt)]
fig, ax = plt.subplots(figsize=(10,9.5))
ax.plot(climb[:,0], climb[:,1], color="#2ca02c", lw=3, label="CLIMB leg 45 deg (1:4->152->300m, %.1f km)"%(CLIMB/1000))
cols = ["#1f77b4","#d62728","#9467bd","#ff7f0e","#1f77b4"]
for i,(nm,s) in enumerate(segs):
    ax.plot(s[:,0], s[:,1], color=cols[i%5], lw=2.4, label=nm if i<4 else None)
ax.plot(desc[:,0], desc[:,1], color="#8c564b", lw=3, ls="--", label="DESCENT return -> HOME (P9 38 + P10, %.1f km)"%(DESC/1000))
ax.plot(0,0,"*",color="gold",ms=26,mec="k",zorder=6,label="HOME (takeoff + land)")
ax.plot(A[0],A[1],"^",color="lime",ms=14,mec="k",zorder=6,label="climb-top (infinity centre)")
ax.set_aspect("equal"); ax.grid(alpha=.3); ax.set_xlabel("East [m]"); ax.set_ylabel("North [m]")
ax.set_title("Mk-7 figure-8 design v2 (distance-budget): climb 45deg -> COMPACT infinity at climb-top -> descent -> HOME\nR=%.0f m, infinity self-closure %.0f m; climb %.1f km / descent %.1f km legs"%(R,selfcl,CLIMB/1000,DESC/1000),fontsize=9.5)
ax.legend(loc="upper left",fontsize=8)
plt.tight_layout(); plt.savefig(out,dpi=125); print("SAVED",out,"| infinity self-closure=%.0f m | infinity centre %.0f m NE of HOME"%(selfcl,np.hypot(*A)))
