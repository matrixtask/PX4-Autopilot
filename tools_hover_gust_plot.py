#!/usr/bin/env python3
# Final plot for the Choshi hover gust test: ONE graph, left axis = XYZ position deviations [m] (+/-2 m band),
# right axis = wind speed u + vertical gust [m/s], x = time since hold start. + CSV. PEAK verdict per axis.
import numpy as np, matplotlib
matplotlib.use("Agg"); import matplotlib.pyplot as plt
from pyulog import ULog
import sys
ULG = sys.argv[1]; T0_MARKER = float(open("/tmp/mk7_wind_go").read().strip())
u = ULog(ULG); D = {d.name: d.data for d in u.data_list}
t0 = D["vehicle_attitude"]["timestamp"][0]
def T(n): return (np.asarray(D[n]["timestamp"], float) - t0) / 1e6
lp = D["vehicle_local_position"]; tl = T("vehicle_local_position")
N = np.asarray(lp["x"]); E = np.asarray(lp["y"]); alt = -np.asarray(lp["z"])
ih = np.where(alt > 49.0)[0][0]; th = tl[ih]           # hold start in ulog time == wall T0_MARKER
hold = tl >= th
tt = tl[hold] - th; dN = N[hold] - N[ih]; dE = E[hold] - E[ih]; dU = alt[hold] - 50.0
# wind playback (wall-clock) -> hold-relative time
W = np.genfromtxt("/tmp/mk7_wind_played.csv", delimiter=",", names=True)
tw = W["wall_t"] - T0_MARKER
uw = W["u"]; wz = W["wz"]
# evaluation: steady window = after the 45 s ramp + 15 s settle, UNTIL the hold stream ends (960 s)
m = (tt >= 60.0) & (tt <= 960.0)
pkN, pkE, pkU = np.abs(dN[m]).max(), np.abs(dE[m]).max(), np.abs(dU[m]).max()
for nm, s in [("dN", dN), ("dE", dE), ("dU", dU)]:
    k = np.argmax(np.abs(s[m])); print("  peak %s=%.2f at t=%.0f s" % (nm, s[m][k], tt[m][k]))
ok = (pkN < 2.0) and (pkE < 2.0) and (pkU < 2.0)
print("STEADY window (60..960 s after hold):")
print("  PEAK |dN|=%.2f  |dE|=%.2f  |dU|=%.2f m  ->  %s (limit 2.0 each)" % (pkN, pkE, pkU, "PASS" if ok else "FAIL"))
print("  wind: mean=%.1f max3s~%.1f m/s" % (uw[tw > 45].mean(), 45.0))
np.savetxt("/tmp/mk7_hover_gust.csv",
           np.column_stack([tt, dN, dE, dU, np.interp(tt, tw, uw), np.interp(tt, tw, wz)]),
           delimiter=",", header="t_s,dN_m,dE_m,dU_m,wind_u_ms,wind_w_ms", comments="", fmt="%.3f")
fig, ax = plt.subplots(figsize=(14, 6.5))
ax.axhspan(-2, 2, color="green", alpha=0.10, label="±2 m target band")
ax.plot(tt, dN, lw=0.8, color="#d62728", label="ΔX North [m]")
ax.plot(tt, dE, lw=0.8, color="#1f77b4", label="ΔY East [m]")
ax.plot(tt, dU, lw=0.8, color="#2ca02c", label="ΔZ Up [m]")
ax.axhline(2, color="green", lw=0.8, ls="--"); ax.axhline(-2, color="green", lw=0.8, ls="--")
ax.set_xlabel("time since hold start [s]"); ax.set_ylabel("position deviation [m]")
ax.set_ylim(-6, 6); ax.grid(alpha=.3)
ax2 = ax.twinx()
ax2.plot(tw, uw, lw=0.7, color="gray", alpha=0.8, label="wind u (along, FROM North) [m/s]")
ax2.plot(tw, wz, lw=0.6, color="tan", alpha=0.7, label="wind w (vertical) [m/s]")
ax2.set_ylabel("wind [m/s]"); ax2.set_ylim(0, 60)
h1, l1 = ax.get_legend_handles_labels(); h2, l2 = ax2.get_legend_handles_labels()
ax.legend(h1 + h2, l1 + l2, loc="upper right", fontsize=8, ncol=2)
ax.set_title("Mk-7 hover gust-resistance (Choshi model): U=30 m/s + AIJ catII von Karman turbulence + (1-cos) 45 m/s gusts @200/500/800 s\n"
             "alt 50 m, nose upwind (N), pusher station-keeping | steady PEAK |dN|=%.2f |dE|=%.2f |dU|=%.2f m -> %s (±2 m)"
             % (pkN, pkE, pkU, "PASS" if ok else "FAIL"), fontsize=10)
plt.tight_layout(); plt.savefig("/tmp/mk7_hover_gust.png", dpi=125)
print("SAVED /tmp/mk7_hover_gust.png + /tmp/mk7_hover_gust.csv")
