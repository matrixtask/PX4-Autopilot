#!/usr/bin/env python3
# Analyse the VERT racetrack run: plan view, landing closure vs HOME, N-S/E-W extent, per-phase PEAK osc.
import numpy as np, matplotlib
matplotlib.use("Agg"); import matplotlib.pyplot as plt
from pyulog import ULog
f=open("/tmp/rpfV01_ulg.txt").read().strip()
u=ULog(f); D={d.name:d.data for d in u.data_list}
t0=D["vehicle_attitude"]["timestamp"][0]
def T(n): return (np.asarray(D[n]["timestamp"],float)-t0)/1e6
lp=D["vehicle_local_position"]; tl=T("vehicle_local_position")
N=np.asarray(lp["x"]); E=np.asarray(lp["y"]); alt=-np.asarray(lp["z"]); V=np.hypot(np.asarray(lp["vx"]),np.asarray(lp["vy"]))
att=D["vehicle_attitude"]; ta=T("vehicle_attitude")
q=np.c_[att["q[0]"],att["q[1]"],att["q[2]"],att["q[3]"]]
roll=np.degrees(np.arctan2(2*(q[:,0]*q[:,1]+q[:,2]*q[:,3]),1-2*(q[:,1]**2+q[:,2]**2)))
pitch=np.degrees(np.arcsin(np.clip(2*(q[:,0]*q[:,2]-q[:,3]*q[:,1]),-1,1)))
hdg=np.degrees(np.arctan2(2*(q[:,0]*q[:,3]+q[:,1]*q[:,2]),1-2*(q[:,2]**2+q[:,3]**2)))%360
tas=np.interp(ta,T("airspeed_validated"),np.asarray(D["airspeed_validated"]["true_airspeed_m_s"]))
alta=np.interp(ta,tl,alt); Na=np.interp(ta,tl,N); Ea=np.interp(ta,tl,E); Va=np.interp(ta,tl,V)
# landing = last sample on ground
iland=np.where(alt<0.6)[0]; iland=iland[-1] if len(iland) else len(alt)-1
land=np.array([E[iland],N[iland]]); closure=np.hypot(*land)
ext_N=N[:iland].max()-N[:iland].min(); ext_E=E[:iland].max()-E[:iland].min()
# detrended peak-to-peak osc helper (remove ~3s moving-average trend -> the wobble)
def pp_osc(sig,tt,m):
    if m.sum()<10: return float("nan")
    s=sig[m]; t=tt[m]; w=max(3,int(len(s)*0.04))
    tr=np.convolve(s,np.ones(w)/w,mode="same"); r=s-tr
    return r.max()-r.min()
# phase masks by alt/hdg/speed (FW portion)
climb=(alta>20)&(alta<288)&(np.gradient(alta,ta)>0.4)
cruise1=(alta>290)&(np.abs(((hdg-30+180)%360)-180)<25)&(tas>64)
brake=(alta>290)&(np.abs(((hdg-210+180)%360)-180)<30)&(tas>44)&(tas<72)
desc=(alta<290)&(alta>158)&(tas>33)&(tas<46)&(np.gradient(alta,ta)<-0.3)
# turn windows by |roll|
banked=np.abs(roll)>15
turn1=banked&(alta>285); turn2=banked&(alta>150)&(alta<165)
print("=== VERT racetrack run analysis ===")
print("LAND (E,N)=(%.0f,%.0f) m  CLOSURE(dist HOME)=%.0f m  [design predict 432]"%(land[0],land[1],closure))
print("extent  N-S=%.0f m  E-W=%.0f m  -> %s"%(ext_N,ext_E,"VERTICAL(N-S long) ✓" if ext_N>ext_E else "WIDE(E-W)"))
print("\nphase            samples  rollPP(osc)  pitchPP(osc)  meanV  altrange")
for nm,m in [("CLIMB",climb),("CRUISE-out(30)",cruise1),("TURN1(CCW)",turn1),("BRAKE(210)",brake),("DESCENT-P9",desc),("TURN2(CCW@152)",turn2)]:
    if m.sum()<5: print("  %-15s (n/a)"%nm); continue
    print("  %-15s %5d   %5.2f deg     %5.2f deg     %4.0f   %4.0f-%4.0f m"%(nm,m.sum(),pp_osc(roll,ta,m),pp_osc(pitch,ta,m),np.mean(tas[m]),alta[m].min(),alta[m].max()))
print("\nplateaus: cruise V=%.1f (tgt72)  descent V=%.1f (tgt38)  apex alt=%.0f (tgt300)  gate=%.0f (tgt152)"%(
    np.mean(tas[cruise1]) if cruise1.sum() else 0, np.mean(tas[desc]) if desc.sum() else 0, alta[cruise1].mean() if cruise1.sum() else 0, np.mean(alta[desc&(alta<160)]) if (desc&(alta<160)).sum() else 152))
# ===== plan view =====
fig,ax=plt.subplots(figsize=(8.5,11))
sc=ax.scatter(E[:iland+1],N[:iland+1],c=alt[:iland+1],cmap="viridis",s=6)
plt.colorbar(sc,ax=ax,label="altitude [m]",shrink=0.6)
ax.plot(0,0,"*",color="gold",ms=26,mec="k",zorder=6,label="HOME (takeoff)")
ax.plot(land[0],land[1],"X",color="red",ms=16,mec="k",zorder=6,label="LAND (closure %.0f m)"%closure)
ax.annotate("",xy=(E[np.argmax(alta>30)]+0,N[np.argmax(alta>30)]+0),xytext=(0,0),arrowprops=dict(arrowstyle="->",lw=1.5,color="gray"))
ax.set_aspect("equal"); ax.grid(alpha=.3); ax.set_xlabel("East [m]"); ax.set_ylabel("North [m]")
ax.set_title("Mk-7 VERT racetrack (rpfV01 MEASURED): out 30deg NNE, CCW180 x2\nN-S=%.1fkm E-W=%.1fkm; LAND closure %.0f m (predict 432)"%(ext_N/1000,ext_E/1000,closure),fontsize=10)
ax.legend(loc="upper right",fontsize=9)
plt.tight_layout(); plt.savefig("/tmp/mk7_vert_planview.png",dpi=125); print("\nSAVED /tmp/mk7_vert_planview.png")
