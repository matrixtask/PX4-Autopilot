#!/usr/bin/env python3
# Raw pitch/yaw angle + rate time history (no smoothing). usage: tools_att_history.py <ulog> <axis=pitch|yaw> <out_png> <out_csv>
import sys, numpy as np, matplotlib
matplotlib.use("Agg"); import matplotlib.pyplot as plt
from pyulog import ULog
ulg, axis, outpng, outcsv = sys.argv[1], sys.argv[2], sys.argv[3], sys.argv[4]
u = ULog(ulg); D = {d.name: d.data for d in u.data_list}
t0 = D["vehicle_attitude"]["timestamp"][0]
def T(n): return (np.asarray(D[n]["timestamp"], float) - t0) / 1e6
def ang(qw, qx, qy, qz):
    if axis == "pitch":
        return np.degrees(np.arcsin(np.clip(2*(qw*qy-qz*qx), -1, 1)))
    else:  # yaw, unwrapped (continuous) for readability
        return np.degrees(np.unwrap(np.arctan2(2*(qw*qz+qx*qy), 1-2*(qy*qy+qz*qz))))
aqd = D["vehicle_attitude"]; taq = T("vehicle_attitude"); qa = [np.asarray(aqd["q[%d]" % i]) for i in range(4)]
n = min(len(taq), *[len(x) for x in qa]); taq = taq[:n]; qw, qx, qy, qz = [x[:n] for x in qa]
angle = ang(qw, qx, qy, qz)
av = D["vehicle_angular_velocity"]; tar = T("vehicle_angular_velocity")
rate = np.degrees(np.asarray(av["xyz[1]" if axis == "pitch" else "xyz[2]"]))
ang_sp = tsp = None
if "vehicle_attitude_setpoint" in D:
    sp = D["vehicle_attitude_setpoint"]; tsp = T("vehicle_attitude_setpoint")
    qd = [np.asarray(sp["q_d[%d]" % i]) for i in range(4)]
    m = min(len(tsp), *[len(x) for x in qd]); tsp = tsp[:m]
    ang_sp = ang(*[x[:m] for x in qd])
lp = D["vehicle_local_position"]; tl = T("vehicle_local_position")
alt = -np.asarray(lp["z"]); V = np.hypot(np.asarray(lp["vx"]), np.asarray(lp["vy"])); yawh = np.degrees(np.asarray(lp["heading"])) % 360
rollf = np.interp(tl, taq, np.degrees(np.arctan2(2*(qw*qx+qy*qz), 1-2*(qx*qx+qy*qy))))
def first(c, a):
    ix = np.where(c & (tl > a))[0]; return tl[ix[0]] if len(ix) else a
t_lift = tl[np.where(alt > 0.5)[0][0]]
t1 = first(alt > 9.3, t_lift); t2 = t1+8.0; t3 = first(np.abs((yawh-90+540) % 360-180) < 2.0, t2)
t4 = first(alt > 151, t3); t5 = first(alt > 298, t4); t6 = first(np.abs(rollf) > 12, t5+3)
t7 = first((np.abs((yawh-270+540) % 360-180) < 12) & (np.abs(rollf) < 8), t6+8)
t8 = first(V < 43, t7+3); t9 = first(alt < 153, t8+3); t10 = first(alt < 11, t9+3); t11 = first(alt < 9.0, t10+5)
itd = np.where((tl > t11) & (alt < 1.0))[0]; ttd = tl[itd[0]] if len(itd) else tl[-1]
PH = [("P1", t_lift, t1), ("P2", t1, t2), ("P3", t2, t3), ("P4", t3, t4), ("P5", t4, t5), ("P6", t5, t6),
      ("P7", t6, t7), ("P8", t7, t8), ("P9", t8, t9), ("P10", t9, t10), ("P11", t10, t11), ("P12", t11, ttd)]
col = "#8c564b" if axis == "pitch" else "#9467bd"
fig, ax = plt.subplots(2, 1, figsize=(15, 8.5), sharex=True)
ax[0].plot(taq, angle, color="#1f77b4", lw=0.8, label="%s (actual)" % axis)
if ang_sp is not None: ax[0].plot(tsp, ang_sp, color="#d62728", lw=0.7, alpha=0.6, label="%s_sp (commanded)" % axis)
ax[0].set_ylabel("%s angle [deg]%s" % (axis, " (unwrapped)" if axis == "yaw" else "")); ax[0].axhline(0, color="#aaa", lw=0.3)
ax[0].legend(loc="upper right", fontsize=9); ax[0].grid(alpha=.3)
ax[1].plot(tar, rate, color=col, lw=0.6); ax[1].axhline(0, color="#aaa", lw=0.3)
ax[1].set_ylabel("%s rate [deg/s] (raw gyro)" % axis); ax[1].set_xlabel("t [s]"); ax[1].grid(alpha=.3)
ymax = max(5, np.percentile(np.abs(rate), 99.9)*1.1); ax[1].set_ylim(-ymax, ymax)
for nm, a, b in PH:
    for axi in ax: axi.axvline(a, color="#888", lw=0.5, ls=":")
    ax[0].text((a+b)/2, ax[0].get_ylim()[1]*0.94, nm, ha="center", fontsize=8, color="#555")
for axi in ax: axi.axvline(ttd, color="#888", lw=0.5, ls=":")
ax[0].set_xlim(t_lift-2, ttd+2)
fig.suptitle("Mk-7 RP-1 %s-angle & %s-rate time history (raw, no smoothing) — %s" % (axis, axis, ulg.split("/")[-1]), fontsize=11)
plt.tight_layout(); plt.savefig(outpng, dpi=120); print("SAVED", outpng)
print("\nphase   %s[deg] min..max   |%s|pk   %s-rate[deg/s] pk" % (axis, axis, axis))
for nm, a, b in PH:
    aw = angle[(taq >= a) & (taq <= b)]; rw = rate[(tar >= a) & (tar <= b)]
    if len(aw) and len(rw):
        print("  %-4s  %8.1f .. %7.1f   %6.1f      %7.1f" % (nm, aw.min(), aw.max(), np.abs(aw).max(), np.abs(rw).max()))
import csv
tg = tar; ang_i = np.interp(tg, taq, angle); sp_i = np.interp(tg, tsp, ang_sp) if ang_sp is not None else np.full_like(tg, np.nan)
with open(outcsv, "w", newline="") as f:
    w = csv.writer(f); w.writerow(["t_s", "%s_deg" % axis, "%s_rate_dps" % axis, "%s_sp_deg" % axis])
    for i in range(len(tg)): w.writerow(["%.4f" % tg[i], "%.4f" % ang_i[i], "%.4f" % rate[i], ("%.4f" % sp_i[i]) if not np.isnan(sp_i[i]) else ""])
print("SAVED", outcsv, "(%d rows)" % len(tg))
