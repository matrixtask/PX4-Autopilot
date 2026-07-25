#!/usr/bin/env python3
# Ground-speed companion plot for the hover gust test: vN/vE/vD + |Vground| (ideal 0 for station-keeping),
# time axis identical to the position-deviation plot (seconds since hold start). Wind u overlaid (right axis)
# so the causality wind input -> ground-speed response -> position deviation reads across the two figures.
import numpy as np, matplotlib, sys
matplotlib.use("Agg"); import matplotlib.pyplot as plt
from pyulog import ULog
ULG = sys.argv[1]; OUT = sys.argv[2] if len(sys.argv) > 2 else "/tmp/mk7_hover_gust_p30_gspeed.png"
T0_MARKER = float(open("/tmp/mk7_wind_go").read().strip())
u = ULog(ULG); D = {d.name: d.data for d in u.data_list}
t0 = D["vehicle_attitude"]["timestamp"][0]
lp = D["vehicle_local_position"]; tl = (np.asarray(lp["timestamp"], float) - t0) / 1e6
alt = -np.asarray(lp["z"])
vN = np.asarray(lp["vx"]); vE = np.asarray(lp["vy"]); vD = np.asarray(lp["vz"])
ih = np.where(alt > 49.0)[0][0]; th = tl[ih]
m = (tl >= th) & (tl <= th + 975.0)
tt = tl[m] - th
vNm, vEm, vDm = vN[m], vE[m], vD[m]
vg = np.sqrt(vNm**2 + vEm**2 + vDm**2)
W = np.genfromtxt("/tmp/mk7_wind_played.csv", delimiter=",", names=True)
tw = W["wall_t"] - T0_MARKER; uw = W["u"]
me = (tt >= 60.0) & (tt <= 960.0)
print("ground-speed steady window (60-960 s): PEAK |vN|=%.2f |vE|=%.2f |vD|=%.2f |Vg|=%.2f m/s ; std %.2f/%.2f/%.2f" %
      (np.abs(vNm[me]).max(), np.abs(vEm[me]).max(), np.abs(vDm[me]).max(), vg[me].max(),
       vNm[me].std(), vEm[me].std(), vDm[me].std()))
fig, ax = plt.subplots(figsize=(14, 6.0))
ax.plot(tt, vNm, lw=0.7, color="#d62728", label="vN North [m/s]")
ax.plot(tt, vEm, lw=0.7, color="#1f77b4", label="vE East [m/s]")
ax.plot(tt, vDm, lw=0.7, color="#2ca02c", label="vD Down [m/s]")
ax.plot(tt, vg, lw=0.9, color="black", alpha=0.55, label="|Vground| [m/s]")
ax.axhline(0, color="gray", lw=0.6)
ax.set_xlabel("time since hold start [s]"); ax.set_ylabel("ground velocity [m/s]")
ax.set_xlim(-20, 975); ax.set_ylim(-4, 4); ax.grid(alpha=.3)
ax2 = ax.twinx()
ax2.plot(tw, uw, lw=0.6, color="gray", alpha=0.7, label="wind u [m/s]")
ax2.set_ylabel("wind u [m/s]"); ax2.set_ylim(0, 40)
h1, l1 = ax.get_legend_handles_labels(); h2, l2 = ax2.get_legend_handles_labels()
ax.legend(h1 + h2, l1 + l2, loc="upper right", fontsize=8, ncol=2)
ax.set_title("Mk-7 hover gust test (PEAK-TOTAL 30): GROUND SPEED (station-keeping ideal = 0) + wind input\n"
             "same time axis as the position-deviation plot; steady peaks vN %.2f / vE %.2f / vD %.2f / |Vg| %.2f m/s"
             % (np.abs(vNm[me]).max(), np.abs(vEm[me]).max(), np.abs(vDm[me]).max(), vg[me].max()), fontsize=10)
plt.tight_layout(); plt.savefig(OUT, dpi=125)
print("SAVED", OUT)
