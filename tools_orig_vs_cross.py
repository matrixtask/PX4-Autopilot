#!/usr/bin/env python3
# Before/after: original single-loop RP-1 (rpf200, default) vs figure-8 CROSS 202 (rpfC03). Overlay plan view + metrics.
import numpy as np, matplotlib
matplotlib.use("Agg"); import matplotlib.pyplot as plt
from pyulog import ULog
def load(f):
    u=ULog(open(f).read().strip()); D={d.name:d.data for d in u.data_list}
    t0=D["vehicle_attitude"]["timestamp"][0]
    lp=D["vehicle_local_position"]; tl=(np.asarray(lp["timestamp"],float)-t0)/1e6
    N=np.asarray(lp["x"]); E=np.asarray(lp["y"]); alt=-np.asarray(lp["z"])
    il=np.where(alt<0.6)[0]; il=il[-1] if len(il) else len(alt)-1
    att=D["vehicle_attitude"]; ta=(np.asarray(att["timestamp"],float)-t0)/1e6
    q=np.c_[att["q[0]"],att["q[1]"],att["q[2]"],att["q[3]"]]
    roll=np.degrees(np.arctan2(2*(q[:,0]*q[:,1]+q[:,2]*q[:,3]),1-2*(q[:,1]**2+q[:,2]**2)))
    # turn blocks (|roll|>15)
    idx=np.where(np.abs(roll)>15)[0]; blocks=np.split(idx,np.where(np.diff(idx)>200)[0]+1) if len(idx) else []
    turns=[("CW(+)" if np.mean(roll[b])>0 else "CCW(-)") for b in blocks if len(b)>40]
    return dict(N=N,E=E,alt=alt,il=il,land=np.array([E[il],N[il]]),clo=np.hypot(E[il],N[il]),
        extE=E[:il].max()-E[:il].min(), extN=N[:il].max()-N[:il].min(), tflight=tl[il], turns=turns)
O=load("/tmp/rpf200_ulg.txt"); C=load("/tmp/rpfC01_ulg.txt")   # rpfC01_ulg.txt currently points at rpfC03
print("=== BEFORE (single-loop RP-1) vs AFTER (CROSS-202) ===")
print("%-22s | %-22s | %-22s"%("metric","BEFORE single-loop","AFTER CROSS-202"))
print("-"*70)
print("%-22s | %-22s | %-22s"%("shape","U (out/turn/back)","figure-8 CROSS"))
print("%-22s | %-22s | %-22s"%("extent E-W [km]","%.1f"%(O["extE"]/1000),"%.1f"%(C["extE"]/1000)))
print("%-22s | %-22s | %-22s"%("extent N-S [km]","%.1f"%(O["extN"]/1000),"%.1f"%(C["extN"]/1000)))
print("%-22s | %-22s | %-22s"%("turns","%d %s"%(len(O["turns"]),O["turns"]),"%d %s"%(len(C["turns"]),C["turns"])))
print("%-22s | %-22s | %-22s"%("land (E,N) [m]","(%.0f,%.0f)"%(O["land"][0],O["land"][1]),"(%.0f,%.0f)"%(C["land"][0],C["land"][1])))
print("%-22s | %-22s | %-22s"%("closure to HOME [m]","%.0f"%O["clo"],"%.0f"%C["clo"]))
print("%-22s | %-22s | %-22s"%("flight time [s]","%.0f"%O["tflight"],"%.0f"%C["tflight"]))
fig,ax=plt.subplots(figsize=(11,8))
ax.plot(O["E"][:O["il"]+1],O["N"][:O["il"]+1],color="#1f77b4",lw=2.2,label="BEFORE: single-loop RP-1 (East out, CW180, U)")
ax.plot(C["E"][:C["il"]+1],C["N"][:C["il"]+1],color="#d62728",lw=2.2,label="AFTER: figure-8 CROSS-202 (45NE out, CW202+CCW202)")
ax.plot(0,0,"*",color="gold",ms=26,mec="k",zorder=6,label="HOME (common takeoff)")
ax.plot(O["land"][0],O["land"][1],"o",color="#1f77b4",ms=13,mec="k",zorder=6,label="BEFORE land (%.0f m)"%O["clo"])
ax.plot(C["land"][0],C["land"][1],"X",color="#d62728",ms=15,mec="k",zorder=6,label="AFTER land (%.0f m)"%C["clo"])
ax.set_aspect("equal"); ax.grid(alpha=.3); ax.set_xlabel("East [m]"); ax.set_ylabel("North [m]")
ax.set_title("Mk-7: original single-loop RP-1  vs  figure-8 CROSS-202  (overlay, common HOME)\nBEFORE E-W %.1f x N-S %.1f km, 1 turn | AFTER N-S %.1f x E-W %.1f km, 2 turns, self-crossing"%(O["extE"]/1000,O["extN"]/1000,C["extN"]/1000,C["extE"]/1000),fontsize=10)
ax.legend(loc="best",fontsize=8.5)
plt.tight_layout(); plt.savefig("/tmp/mk7_orig_vs_cross.png",dpi=125); print("\nSAVED /tmp/mk7_orig_vs_cross.png")
