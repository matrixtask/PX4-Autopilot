#!/usr/bin/env python3
# Raw roll-angle + roll-rate time history for the whole mission (no smoothing). usage: tools_roll_history.py <ulog> <out_png> <out_csv>
import sys, numpy as np, matplotlib
matplotlib.use("Agg"); import matplotlib.pyplot as plt
from pyulog import ULog
ulg, outpng, outcsv = sys.argv[1], sys.argv[2], sys.argv[3]
u = ULog(ulg); D = {d.name: d.data for d in u.data_list}
t0 = D["vehicle_attitude"]["timestamp"][0]
def T(n): return (np.asarray(D[n]["timestamp"], float) - t0) / 1e6
# actual roll from attitude quaternion (length-guarded for multi-instance logs)
aqd = D["vehicle_attitude"]; taq = T("vehicle_attitude"); qa = [np.asarray(aqd["q[%d]" % i]) for i in range(4)]
n = min(len(taq), *[len(x) for x in qa]); taq = taq[:n]; qw, qx, qy, qz = [x[:n] for x in qa]
roll = np.degrees(np.arctan2(2*(qw*qx+qy*qz), 1-2*(qx*qx+qy*qy)))
# raw body roll rate (gyro, ~250 Hz) in deg/s
av = D["vehicle_angular_velocity"]; tar = T("vehicle_angular_velocity"); rollrate = np.degrees(np.asarray(av["xyz[0]"]))
# commanded roll from attitude setpoint q_d, if present
roll_sp = tsp = None
if "vehicle_attitude_setpoint" in D:
    sp = D["vehicle_attitude_setpoint"]; tsp = T("vehicle_attitude_setpoint")
    qd = [np.asarray(sp["q_d[%d]" % i]) for i in range(4)]
    m = min(len(tsp), *[len(x) for x in qd]); tsp = tsp[:m]; sw, sx, sy, sz = [x[:m] for x in qd]
    roll_sp = np.degrees(np.arctan2(2*(sw*sx+sy*sz), 1-2*(sx*sx+sy*sy)))
# ---- physical phase boundaries (same milestones as make_compliance.py) ----
lp = D["vehicle_local_position"]; tl = T("vehicle_local_position")
alt = -np.asarray(lp["z"]); V = np.hypot(np.asarray(lp["vx"]), np.asarray(lp["vy"])); yaw = np.degrees(np.asarray(lp["heading"])) % 360
rollf = np.interp(tl, taq, roll)
def first(c, a):
    ix = np.where(c & (tl > a))[0]; return tl[ix[0]] if len(ix) else a
t_lift = tl[np.where(alt > 0.5)[0][0]]
t1 = first(alt > 9.3, t_lift); t2 = t1 + 8.0
t3 = first(np.abs((yaw-90+540) % 360-180) < 2.0, t2)
t4 = first(alt > 151, t3); t5 = first(alt > 298, t4)
t6 = first(np.abs(rollf) > 12, t5+3); t7 = first((np.abs((yaw-270+540) % 360-180) < 12) & (np.abs(rollf) < 8), t6+8)
t8 = first(V < 43, t7+3); t9 = first(alt < 153, t8+3); t10 = first(alt < 11, t9+3); t11 = first(alt < 9.0, t10+5)
itd = np.where((tl > t11) & (alt < 1.0))[0]; ttd = tl[itd[0]] if len(itd) else tl[-1]
PH = [("P1", t_lift, t1), ("P2", t1, t2), ("P3", t2, t3), ("P4", t3, t4), ("P5", t4, t5), ("P6", t5, t6),
      ("P7", t6, t7), ("P8", t7, t8), ("P9", t8, t9), ("P10", t9, t10), ("P11", t10, t11), ("P12", t11, ttd)]
bounds = [t_lift, t1, t2, t3, t4, t5, t6, t7, t8, t9, t10, t11, ttd]
# ---- figure ----
fig, ax = plt.subplots(2, 1, figsize=(15, 8.5), sharex=True)
ax[0].plot(taq, roll, color="#1f77b4", lw=0.8, label="roll (actual)")
if roll_sp is not None: ax[0].plot(tsp, roll_sp, color="#d62728", lw=0.7, alpha=0.6, label="roll_sp (commanded)")
ax[0].set_ylabel("roll angle [deg]"); ax[0].axhline(0, color="#aaa", lw=0.3); ax[0].legend(loc="upper right", fontsize=9); ax[0].grid(alpha=.3)
ax[1].plot(tar, rollrate, color="#2ca02c", lw=0.6); ax[1].axhline(0, color="#aaa", lw=0.3)
ax[1].set_ylabel("roll rate [deg/s]  (raw gyro)"); ax[1].set_xlabel("t [s]"); ax[1].grid(alpha=.3)
ymax = max(5, np.percentile(np.abs(rollrate), 99.9) * 1.1); ax[1].set_ylim(-ymax, ymax)
for nm, a, b in PH:
    for axi in ax: axi.axvline(a, color="#888", lw=0.5, ls=":")
    ax[0].text((a+b)/2, ax[0].get_ylim()[1]*0.94, nm, ha="center", fontsize=8, color="#555")
ax[0].axvline(ttd, color="#888", lw=0.5, ls=":"); ax[1].axvline(ttd, color="#888", lw=0.5, ls=":")
ax[0].set_xlim(t_lift-2, ttd+2)
fig.suptitle("Mk-7 RP-1 roll-angle & roll-rate time history (raw, no smoothing) — %s" % ulg.split("/")[-1], fontsize=11)
plt.tight_layout(); plt.savefig(outpng, dpi=120); print("SAVED", outpng)
# ---- per-phase RAW peaks ----
print("\nphase   roll[deg] min..max  |roll|pk   roll-rate[deg/s] pk")
for nm, a, b in PH:
    rw = roll[(taq >= a) & (taq <= b)]; rrw = rollrate[(tar >= a) & (tar <= b)]
    if len(rw) and len(rrw):
        print("  %-4s  %7.1f .. %6.1f   %6.1f      %7.1f" % (nm, rw.min(), rw.max(), np.abs(rw).max(), np.abs(rrw).max()))
# ---- CSV on the gyro grid (raw rate preserved; roll/roll_sp interpolated) ----
tg = tar
roll_i = np.interp(tg, taq, roll)
rsp_i = np.interp(tg, tsp, roll_sp) if roll_sp is not None else np.full_like(tg, np.nan)
import csv
with open(outcsv, "w", newline="") as f:
    w = csv.writer(f); w.writerow(["t_s", "roll_deg", "roll_rate_dps", "roll_sp_deg"])
    for i in range(len(tg)): w.writerow(["%.4f" % tg[i], "%.4f" % roll_i[i], "%.4f" % rollrate[i], ("%.4f" % rsp_i[i]) if not np.isnan(rsp_i[i]) else ""])
print("SAVED", outcsv, "(%d rows)" % len(tg))
