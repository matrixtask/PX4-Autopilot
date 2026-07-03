#!/usr/bin/env python3
# Analyse the CROSS-202 run: plan view + self-cross detection, closure, CW/CCW turn signs, speeds/alts, per-phase PEAK osc.
import numpy as np, matplotlib
matplotlib.use("Agg"); import matplotlib.pyplot as plt
from pyulog import ULog
u=ULog(open("/tmp/rpfC01_ulg.txt").read().strip()); D={d.name:d.data for d in u.data_list}
t0=D["vehicle_attitude"]["timestamp"][0]
def T(n): return (np.asarray(D[n]["timestamp"],float)-t0)/1e6
lp=D["vehicle_local_position"]; tl=T("vehicle_local_position")
N=np.asarray(lp["x"]); E=np.asarray(lp["y"]); alt=-np.asarray(lp["z"])
att=D["vehicle_attitude"]; ta=T("vehicle_attitude")
q=np.c_[att["q[0]"],att["q[1]"],att["q[2]"],att["q[3]"]]
roll=np.degrees(np.arctan2(2*(q[:,0]*q[:,1]+q[:,2]*q[:,3]),1-2*(q[:,1]**2+q[:,2]**2)))
pitch=np.degrees(np.arcsin(np.clip(2*(q[:,0]*q[:,2]-q[:,3]*q[:,1]),-1,1)))
hdg=np.degrees(np.arctan2(2*(q[:,0]*q[:,3]+q[:,1]*q[:,2]),1-2*(q[:,2]**2+q[:,3]**2)))%360
tas=np.interp(ta,T("airspeed_validated"),np.asarray(D["airspeed_validated"]["true_airspeed_m_s"]))
alta=np.interp(ta,tl,alt)
iland=np.where(alt<0.6)[0]; iland=iland[-1] if len(iland) else len(alt)-1
land=np.array([E[iland],N[iland]]); closure=np.hypot(*land)
ext_N=N[:iland].max()-N[:iland].min(); ext_E=E[:iland].max()-E[:iland].min()
# self-cross detection on the flown plan path (downsample, ignore near-in-time segments)
def seg_int(p,r,q2,s):
    rxs=r[0]*s[1]-r[1]*s[0]
    if abs(rxs)<1e-6: return None
    t=((q2[0]-p[0])*s[1]-(q2[1]-p[1])*s[0])/rxs; uu=((q2[0]-p[0])*r[1]-(q2[1]-p[1])*r[0])/rxs
    if 0<t<1 and 0<uu<1: return p+t*r
    return None
step=max(1,len(E)//600); PE=E[:iland:step]; PN=N[:iland:step]
Xpt=None
for i in range(len(PE)-1):
    for j in range(i+6,len(PE)-1):
        x=seg_int(np.array([PE[i],PN[i]]),np.array([PE[i+1]-PE[i],PN[i+1]-PN[i]]),np.array([PE[j],PN[j]]),np.array([PE[j+1]-PE[j],PN[j+1]-PN[j]]))
        if x is not None: Xpt=x; break
    if Xpt is not None: break
def pp(sig,m):
    if m.sum()<10: return float("nan")
    s=sig[m]; w=max(5,int(len(s)*0.05)); tr=np.convolve(s,np.ones(w)/w,mode="same"); r=(s-tr)[w:-w]
    return r.max()-r.min() if len(r)>5 else float("nan")
banked=np.abs(roll)>15
# turn1 = first banked block, turn2 = second; check sign
idx=np.where(banked)[0]
gaps=np.where(np.diff(idx)>200)[0]
blocks=np.split(idx,gaps+1) if len(idx) else []
print("=== CROSS-202 run analysis ===")
print("LAND (E,N)=(%.0f,%.0f)  CLOSURE=%.0f m  [predict ~0; home leg log err 3749]"%(land[0],land[1],closure))
print("extent N-S=%.0f E-W=%.0f m ; self-cross(plan)=%s"%(ext_N,ext_E,("YES at (E,N)=(%.0f,%.0f)"%(Xpt[0],Xpt[1])) if Xpt is not None else "NONE detected"))
for k,b in enumerate(blocks[:2]):
    mr=roll[b]; sign="CW(+right bank)" if np.mean(mr)>0 else "CCW(-left bank)"
    print("  TURN%d: mean roll=%+.0f deg -> %s ; |roll|max=%.0f ; hdg %.0f->%.0f"%(k+1,np.mean(mr),sign,np.abs(mr).max(),hdg[b[0]],hdg[b[-1]]))
climb=(alta>20)&(alta<288)&(np.gradient(alta,ta)>0.4)
cruise=(alta>296)&(tas>68)&(np.abs(roll)<8)
desc=(alta<285)&(alta>170)&(tas>35)&(tas<42)&(np.gradient(alta,ta)<-0.3)
print("\nphase       rollPP  pitchPP  meanV  altrange")
for nm,m in [("CRUISE72",cruise),("DESCENT-P9",desc)]:
    print("  %-11s %5.2f   %5.2f    %4.0f   %4.0f-%4.0f"%(nm,pp(roll,m),pp(pitch,m),np.mean(tas[m]) if m.sum() else 0,alta[m].min() if m.sum() else 0,alta[m].max() if m.sum() else 0))
print("plateaus: cruise V=%.1f(72) descentV=%.1f(38) apex=%.0f(300) gate~152 land=%.0f(10->0)"%(np.mean(tas[cruise]) if cruise.sum() else 0,np.mean(tas[desc]) if desc.sum() else 0,alta[cruise].mean() if cruise.sum() else 0,alt[iland]))
fig,ax=plt.subplots(figsize=(9.5,10))
sc=ax.scatter(E[:iland+1],N[:iland+1],c=alt[:iland+1],cmap="viridis",s=6)
plt.colorbar(sc,ax=ax,label="altitude [m]",shrink=0.6)
ax.plot(0,0,"*",color="gold",ms=26,mec="k",zorder=6,label="HOME (takeoff)")
ax.plot(land[0],land[1],"X",color="red",ms=16,mec="k",zorder=6,label="LAND (closure %.0f m)"%closure)
if Xpt is not None: ax.plot(Xpt[0],Xpt[1],"o",color="black",ms=15,mfc="none",mew=2.5,zorder=7,label="self-CROSS")
ax.set_aspect("equal"); ax.grid(alpha=.3); ax.set_xlabel("East [m]"); ax.set_ylabel("North [m]")
ax.set_title("Mk-7 CROSS-202 (rpfC01 MEASURED): 45 NE out, CW202 + CCW202, HOME-aim descent\nN-S=%.1fkm E-W=%.1fkm; self-cross=%s; LAND closure %.0f m"%(ext_N/1000,ext_E/1000,"YES" if Xpt is not None else "NO",closure),fontsize=9.5)
ax.legend(loc="best",fontsize=8)
plt.tight_layout(); plt.savefig("/tmp/mk7_cross202_planview.png",dpi=125); print("\nSAVED /tmp/mk7_cross202_planview.png")
