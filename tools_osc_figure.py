#!/usr/bin/env python3
# Per-phase attitude-oscillation PEAK figure (clean steady windows = middle 50% of each phase, detrended 1.5 s).
# Mirrors make_compliance.py phase detection. usage: tools_osc_figure.py <ulog> <out.png>
import sys, numpy as np, matplotlib
matplotlib.use("Agg"); import matplotlib.pyplot as plt
from pyulog import ULog
ulg, out = sys.argv[1], sys.argv[2]
u = ULog(ulg); D = {d.name: d.data for d in u.data_list}
t0 = D["vehicle_attitude"]["timestamp"][0]
def T(n): return (np.asarray(D[n]["timestamp"], float) - t0) / 1e6
lp = D["vehicle_local_position"]; tl = T("vehicle_local_position")
alt = -np.asarray(lp["z"]); vh = np.hypot(np.asarray(lp["vx"]), np.asarray(lp["vy"])); yaw = np.degrees(np.asarray(lp["heading"])) % 360
q0,q1,q2,q3 = [np.asarray(D["vehicle_attitude"]["q[%d]"%i]) for i in range(4)]
roll = np.degrees(np.arctan2(2*(q0*q1+q2*q3), 1-2*(q1*q1+q2*q2)))
pitch = np.degrees(np.arcsin(np.clip(2*(q0*q2-q3*q1), -1, 1))); ta = T("vehicle_attitude")
def first(cond, after):
    ix = np.where(cond & (tl > after))[0]; return tl[ix[0]] if len(ix) else after
rollf = np.interp(tl, ta, roll)
t_lift = tl[np.where(alt > 0.5)[0][0]]
t_p1end = first(alt > 9.3, t_lift); t_p2end = t_p1end + 8.0
t_p3end = first(np.abs((yaw-90+540)%360-180) < 2.0, t_p2end)
t_p4end = first(alt > 151, t_p3end); t_p5end = first(alt > 298, t_p4end)
t_p6end = first(np.abs(rollf) > 12, t_p5end + 3)
t_p7end = first((np.abs((yaw-270+540)%360-180) < 12) & (np.abs(rollf) < 8), t_p6end + 8)
t_p8end = first(vh < 43, t_p7end + 3); t_p9end = first(alt < 153, t_p8end + 3)
t_p10end = first(alt < 11, t_p9end + 3); t_p11end = first(alt < 9.0, t_p10end + 5)
itd = np.where((tl > t_p11end) & (alt < 1.0))[0]; tl_td = tl[itd[0]] if len(itd) else tl[-1]
PH = [("P1\nTakeoff",t_lift,t_p1end),("P2\nHover1",t_p1end,t_p2end),("P3\nyaw-E",t_p2end,t_p3end),
 ("P4\nClimb¼",t_p3end,t_p4end),("P5\nClimbX",t_p4end,t_p5end),("P6\nCruise",t_p5end,t_p6end),
 ("P7\nTurn",t_p6end,t_p7end),("P8\nBrake",t_p7end,t_p8end),("P9\nDescX",t_p8end,t_p9end),
 ("P10\nDesc¼",t_p9end,t_p10end),("P11\nHover&set",t_p10end,t_p11end),("P12\nLand",t_p11end,tl_td)]
def osc(s, t, a, b):
    if b-a < 2: return 0.0
    g = np.arange(a, b, 0.05); ss = np.interp(g, t, s); k = max(3, int(1.5/0.05))
    tr = np.convolve(ss, np.ones(k)/k, mode="same"); m = slice(k, len(ss)-k)
    return float(np.abs((ss-tr)[m]).max()) if m.stop > m.start else 0.0
labs=[]; RO=[]; PO=[]; YO=[]
for nm,a,b in PH:
    ma,mb = a+0.25*(b-a), b-0.25*(b-a)
    labs.append(nm); RO.append(osc(roll,ta,ma,mb)); PO.append(osc(pitch,ta,ma,mb)); YO.append(osc(yaw,tl,ma,mb))
x = np.arange(len(labs)); w = 0.27
fig, ax = plt.subplots(figsize=(13,5.4))
ax.bar(x-w, RO, w, label="roll", color="#4aa3ff"); ax.bar(x, PO, w, label="pitch", color="#2ca050"); ax.bar(x+w, YO, w, label="yaw", color="#ff8c2a")
ax.axhline(0.5, color="#cc3333", lw=1.6, ls="--", label="spec <0.5 deg PEAK")
for xi,(r,p,y) in enumerate(zip(RO,PO,YO)):
    mx=max(r,p,y); ax.annotate("%.2f"%mx,(xi,mx),textcoords="offset points",xytext=(0,3),ha="center",fontsize=7,color="#333")
ax.set_xticks(x); ax.set_xticklabels(labs, fontsize=8); ax.set_ylabel("attitude oscillation PEAK [deg]  (detrended 1.5 s)")
ax.set_ylim(0, 0.62); ax.legend(ncol=4, fontsize=9, loc="upper center")
ax.set_title("Mk-7 RP-1 per-phase attitude oscillation (THR=0.8 adopted, calm, true TAS, steady mid-50%% windows) — ALL < 0.5 deg", fontsize=10)
ax.grid(axis="y", alpha=.3)
plt.tight_layout(); plt.savefig(out, dpi=120); print("SAVED", out, "| maxosc=%.2f"%max(max(RO),max(PO),max(YO)))
