#!/usr/bin/env python3
# Figure-8 (infinity) DESIGN sketch: HOME-centred, East lobe CW + West lobe CCW, crossing at HOME, cruise legs.
import numpy as np, matplotlib
matplotlib.use("Agg"); import matplotlib.pyplot as plt
out = "/tmp/mk7_fig8_design.png"
# turn radius at 72 m/s, 45 deg bank: R = V^2/(g tan45)
V=72.0; R=V*V/(9.81*np.tan(np.radians(45))); print("turn radius R=%.0f m at 72 m/s / 45 deg bank"%R)

# Build the path from primitives: straight cruise + circular arc.
def cruise(p, hdg, L, n=40):
    h=np.radians(hdg); e=p[0]+np.linspace(0,L,n)*np.sin(h); nn=p[1]+np.linspace(0,L,n)*np.cos(h)
    return np.c_[e,nn]
def arc(p, hdg0, turn, R, n=80):   # turn>0 = CW (right), <0 = CCW (left); returns path
    h0=np.radians(hdg0); s=np.sign(turn); cx=p[0]+s*R*np.cos(h0); cy=p[1]-s*R*np.sin(h0)
    a0=np.arctan2(p[1]-cy,p[0]-cx); aa=a0 - s*np.radians(abs(turn))*np.linspace(0,1,n)
    return np.c_[cx+R*np.cos(aa), cy+R*np.sin(aa)]
def endpt(path): return path[-1]
def endhdg(path):
    d=path[-1]-path[-2]; return np.degrees(np.arctan2(d[0],d[1]))%360

# Symmetric figure-8, 90deg crossing at HOME, lobes = 270deg teardrops. Corners at distance D=R on the diagonals.
# HOME -> NE-corner (cruise NE) -> East lobe CW270 -> SE-corner -> cruise NW through HOME -> NW-corner -> West lobe CCW270 -> SW-corner -> cruise NE -> HOME.
HOME=np.array([0.0,0.0]); EXT=float(__import__("os").environ.get("FIG8_EXT","0.0"))   # extra straight leg into each lobe (0 = compact)
segs=[]; p=HOME.copy(); h=45.0
s=cruise(p,h,R+EXT); segs.append(("P6 cruise out (NE)",s)); p=endpt(s)
s=arc(p,h,270,R); segs.append(("P7 Turn A: CW 270 = LOBE 1 (East)",s)); p=endpt(s); h=endhdg(s)
s=cruise(p,h,2*(R+EXT)); segs.append(("[INSERT P7a] cruise: cross HOME",s)); p=endpt(s)
s=arc(p,h,-270,R); segs.append(("[INSERT P7b] Turn B: CCW 270 = LOBE 2 (West)",s)); p=endpt(s); h=endhdg(s)
s=cruise(p,h,R+EXT); segs.append(("[INSERT P7c] cruise -> HOME (close)",s)); p=endpt(s)
print("path end = (%.0f, %.0f) m  (HOME=0,0 -> closure error %.0f m); heading sequence ends %.0f"%(p[0],p[1],np.hypot(*p),h))

fig,ax=plt.subplots(figsize=(9.5,9))
cols=["#1f77b4","#d62728","#2ca02c","#9467bd","#ff7f0e"]
for i,(nm,s) in enumerate(segs):
    ax.plot(s[:,0],s[:,1],color=cols[i%5],lw=2.4,label=nm)
    mid=s[len(s)//2]; ax.annotate("",xy=s[len(s)//2+1],xytext=s[len(s)//2-1],arrowprops=dict(arrowstyle="-|>",color=cols[i%5],lw=2))
ax.plot(0,0,"*",color="gold",ms=26,mec="k",zorder=6,label="HOME (takeoff + land + crossing)")
ax.plot(0,0,"o",color="k",ms=4,zorder=7)
ax.set_aspect("equal"); ax.grid(alpha=.3); ax.set_xlabel("East [m]"); ax.set_ylabel("North [m]")
ax.set_title("Mk-7 RP-1 figure-8 DESIGN (300 m / 72 TAS): East lobe CW + West lobe CCW, crossing at HOME\nturn R=%.0f m; lobes via 270 deg teardrops + crossing cruise legs; closes at HOME"%R,fontsize=10)
ax.legend(loc="upper left",fontsize=8.5)
plt.tight_layout(); plt.savefig(out,dpi=125); print("SAVED",out)
