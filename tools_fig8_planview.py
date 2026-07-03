#!/usr/bin/env python3
# Figure-8 plan view + geometry analysis. usage: tools_fig8_planview.py <ulog> <out_png>
import sys, numpy as np, matplotlib
matplotlib.use("Agg"); import matplotlib.pyplot as plt
from matplotlib.collections import LineCollection
from pyulog import ULog
ulg, out = sys.argv[1], sys.argv[2]
u = ULog(ulg); D = {d.name: d.data for d in u.data_list}
t0 = D["vehicle_attitude"]["timestamp"][0]
def T(n): return (np.asarray(D[n]["timestamp"], float) - t0) / 1e6
lp = D["vehicle_local_position"]; tl = T("vehicle_local_position")
N = np.asarray(lp["x"]); E = np.asarray(lp["y"]); alt = -np.asarray(lp["z"])
fly = alt > 0.5; Ef, Nf, af, tf = E[fly], N[fly], alt[fly], tl[fly]
# the figure-8 = the 300 m segment (between climb-top and the descent)
m8 = af > 290
i8 = np.where(m8)[0]
fig, ax = plt.subplots(1, 2, figsize=(15, 8))
# LEFT: full path coloured by time
pts = np.array([Ef, Nf]).T.reshape(-1, 1, 2); segs = np.concatenate([pts[:-1], pts[1:]], axis=1)
lc = LineCollection(segs, cmap="turbo", lw=1.8); lc.set_array(tf[:-1]); ax[0].add_collection(lc)
fig.colorbar(lc, ax=ax[0], label="time [s]", fraction=0.046, pad=0.04)
ax[0].plot(0, 0, "*", color="gold", ms=24, mec="k", zorder=6, label="HOME (takeoff)")
ax[0].plot(Ef[-1], Nf[-1], "s", color="red", ms=11, mec="k", zorder=6, label="LAND")
if len(i8):
    ax[0].plot(Ef[i8[0]], Nf[i8[0]], "^", color="lime", ms=13, mec="k", zorder=6, label="fig-8 entry")
    ax[0].plot(Ef[i8[-1]], Nf[i8[-1]], "v", color="magenta", ms=13, mec="k", zorder=6, label="fig-8 close")
ax[0].set_aspect("equal"); ax[0].grid(alpha=.3); ax[0].set_xlabel("East [m]"); ax[0].set_ylabel("North [m]")
ax[0].set_title("FULL path (colour=time): takeoff -> cruise-out -> figure-8 -> return -> land\n%s" % ulg.split("/")[-1], fontsize=10)
ax[0].legend(loc="upper left", fontsize=8)
# RIGHT: zoom on the figure-8 only
if len(i8):
    E8, N8, t8 = Ef[i8], Nf[i8], tf[i8]
    p8 = np.array([E8, N8]).T.reshape(-1, 1, 2); s8 = np.concatenate([p8[:-1], p8[1:]], axis=1)
    lc2 = LineCollection(s8, cmap="turbo", lw=2.2); lc2.set_array(t8[:-1]); ax[1].add_collection(lc2)
    ax[1].plot(E8[0], N8[0], "^", color="lime", ms=14, mec="k", zorder=6, label="entry")
    ax[1].plot(E8[-1], N8[-1], "v", color="magenta", ms=14, mec="k", zorder=6, label="close")
    ax[1].set_aspect("equal"); ax[1].grid(alpha=.3); ax[1].set_xlabel("East [m]"); ax[1].set_ylabel("North [m]")
    ax[1].set_title("FIGURE-8 only (alt>290): is it a symmetric infinity?", fontsize=10); ax[1].legend(fontsize=8)
    # self-closure: distance entry->close
    selfcl = np.hypot(E8[-1]-E8[0], N8[-1]-N8[0])
    cen_e, cen_n = (E8.min()+E8.max())/2, (N8.min()+N8.max())/2
    print("fig-8 entry=(%.0f,%.0f)  close=(%.0f,%.0f)  SELF-closure err=%.0f m"%(E8[0],N8[0],E8[-1],N8[-1],selfcl))
    print("fig-8 extent: E %.0f..%.0f (span %.0f)  N %.0f..%.0f (span %.0f)  centre=(%.0f,%.0f)"%(E8.min(),E8.max(),E8.max()-E8.min(),N8.min(),N8.max(),N8.max()-N8.min(),cen_e,cen_n))
    print("fig-8 centre dist from HOME(0,0) = %.0f m"%np.hypot(cen_e,cen_n))
print("LAND point=(%.0f,%.0f)  dist from HOME=%.0f m"%(Ef[-1],Nf[-1],np.hypot(Ef[-1],Nf[-1])))
plt.tight_layout(); plt.savefig(out, dpi=125); print("SAVED", out)
