#!/usr/bin/env python3
# Top-down plan view (East-North) + side view, colored by altitude. usage: tools_planview.py <ulog> <out_png>
import sys, numpy as np, matplotlib
matplotlib.use("Agg"); import matplotlib.pyplot as plt
from matplotlib.collections import LineCollection
from pyulog import ULog
ulg, out = sys.argv[1], sys.argv[2]
u = ULog(ulg); D = {d.name: d.data for d in u.data_list}
t0 = D["vehicle_attitude"]["timestamp"][0]
def T(n): return (np.asarray(D[n]["timestamp"], float) - t0) / 1e6
lp = D["vehicle_local_position"]; tl = T("vehicle_local_position")
N = np.asarray(lp["x"]); E = np.asarray(lp["y"]); alt = -np.asarray(lp["z"]); V = np.hypot(np.asarray(lp["vx"]), np.asarray(lp["vy"]))
yawh = np.degrees(np.asarray(lp["heading"])) % 360
aqd = D["vehicle_attitude"]; taq = T("vehicle_attitude"); qa = [np.asarray(aqd["q[%d]" % i]) for i in range(4)]
nn = min(len(taq), *[len(x) for x in qa]); qw, qx, qy, qz = [x[:nn] for x in qa]
rollf = np.interp(tl, taq[:nn], np.degrees(np.arctan2(2*(qw*qx+qy*qz), 1-2*(qx*qx+qy*qy))))
def first(c, a):
    ix = np.where(c & (tl > a))[0]; return tl[ix[0]] if len(ix) else a
t_lift = tl[np.where(alt > 0.5)[0][0]]
t1 = first(alt > 9.3, t_lift); t2 = t1+8; t3 = first(np.abs((yawh-90+540) % 360-180) < 2.0, t2)
t4 = first(alt > 151, t3); t5 = first(alt > 298, t4); t6 = first(np.abs(rollf) > 12, t5+3)
t7 = first((np.abs((yawh-270+540) % 360-180) < 12) & (np.abs(rollf) < 8), t6+8)
t8 = first(V < 43, t7+3); t9 = first(alt < 153, t8+3); t10 = first(alt < 11, t9+3); t11 = first(alt < 9.0, t10+5)
itd = np.where((tl > t11) & (alt < 1.0))[0]; ttd = tl[itd[0]] if len(itd) else tl[-1]
MARK = [("P1 takeoff", t_lift), ("P3 yaw-E", t3), ("P4 climb", t4), ("P5 climbX", t5),
        ("P6 cruise E", (t5+t6)/2), ("P7 turn", t6), ("P8 brake", t7), ("P9 descend", t8),
        ("P10 desc1/4", t9), ("P11 hover", t10), ("P12 land", t11)]
m = (tl >= t_lift) & (tl <= ttd+1)
Em, Nm, am, tm = E[m], N[m], alt[m], tl[m]
fig = plt.figure(figsize=(15, 8))
ax = fig.add_subplot(1, 2, 1); axs = fig.add_subplot(1, 2, 2)
# plan view colored by altitude
pts = np.array([Em, Nm]).T.reshape(-1, 1, 2); segs = np.concatenate([pts[:-1], pts[1:]], axis=1)
lc = LineCollection(segs, cmap="viridis", linewidth=2.2); lc.set_array(am[:-1]); ax.add_collection(lc)
cb = fig.colorbar(lc, ax=ax, label="altitude [m]", fraction=0.046, pad=0.04)
ax.plot(Em[0], Nm[0], "o", color="lime", ms=12, mec="k", label="start", zorder=5)
ax.plot(Em[-1], Nm[-1], "s", color="red", ms=11, mec="k", label="end", zorder=5)
for nm, tt in MARK:
    e, n = np.interp(tt, tl, E), np.interp(tt, tl, N)
    ax.plot(e, n, "^", color="white", mec="k", ms=8, zorder=4)
    ax.annotate(nm, (e, n), textcoords="offset points", xytext=(7, 4), fontsize=8, color="#222",
                bbox=dict(boxstyle="round,pad=0.15", fc="white", ec="none", alpha=0.7))
ax.set_xlabel("East [m]"); ax.set_ylabel("North [m]"); ax.set_aspect("equal"); ax.grid(alpha=.3)
ax.set_title("PLAN VIEW (top-down) — East out -> CW turn -> West back loop\n%s" % ulg.split("/")[-1], fontsize=10)
ax.legend(loc="upper right", fontsize=9)
# side view East vs altitude
lc2 = LineCollection(np.concatenate([np.array([Em, am]).T.reshape(-1, 1, 2)[:-1], np.array([Em, am]).T.reshape(-1, 1, 2)[1:]], axis=1), cmap="plasma", linewidth=2.0)
lc2.set_array(np.interp(tm[:-1], tl, V)); axs.add_collection(lc2)
fig.colorbar(lc2, ax=axs, label="V_TAS-equiv (ground spd) [m/s]", fraction=0.046, pad=0.04)
axs.plot(Em[0], am[0], "o", color="lime", ms=11, mec="k"); axs.plot(Em[-1], am[-1], "s", color="red", ms=10, mec="k")
for nm, tt in MARK:
    axs.plot(np.interp(tt, tl, E), np.interp(tt, tl, alt), "^", color="white", mec="k", ms=7)
axs.axhline(300, color="#888", lw=.4, ls=":"); axs.axhline(152, color="#888", lw=.4, ls=":")
axs.set_xlabel("East [m]"); axs.set_ylabel("altitude [m]"); axs.grid(alpha=.3)
axs.set_title("SIDE VIEW (East-altitude) — climb 300 / descend 152 / land", fontsize=10)
plt.tight_layout(); plt.savefig(out, dpi=125); print("SAVED", out, "| extent E %.0f..%.0f N %.0f..%.0f"%(Em.min(),Em.max(),Nm.min(),Nm.max()))
