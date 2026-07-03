#!/usr/bin/env python3
# P9 descend: measured sink-rate(t) and V_TAS(t) overlaid on the IDEAL (38 flat, smooth low-peak sink).
# usage: tools_make_p9_vs_ideal.py <out.png> <label1> <ulog1> [<label2> <ulog2> ...]
import sys, numpy as np, matplotlib
matplotlib.use("Agg"); import matplotlib.pyplot as plt
from pyulog import ULog

out = sys.argv[1]; runs = []
a = sys.argv[2:]
for i in range(0, len(a), 2):
    runs.append((a[i], a[i+1]))

def P9(u):
    u = np.clip(u, 0, 1); return 70*u**9 - 315*u**8 + 540*u**7 - 420*u**6 + 126*u**5

def p9_series(ulg):
    u = ULog(ulg); D = {d.name: d.data for d in u.data_list}
    t0 = D["vehicle_attitude"]["timestamp"][0]
    def T(n): return (np.asarray(D[n]["timestamp"], float) - t0) / 1e6
    lp = D["vehicle_local_position"]; tl = T("vehicle_local_position")
    alt = -np.asarray(lp["z"]); vz = np.asarray(lp["vz"])
    av = D["airspeed_validated"]; tav = T("airspeed_validated"); tas = np.asarray(av["true_airspeed_m_s"])
    # P9 descend = from alt~298 (descend start) to alt~153 (before back-trans), vz>0
    desc = np.where((tl > 100) & (alt < 299) & (alt > 153) & (vz > 0.2))[0]
    # restrict to a contiguous descending run (the P9, not earlier dips)
    a0, a1 = tl[desc[0]], tl[desc[-1]]
    w = (tl >= a0) & (tl <= a1)
    return tl[w] - a0, np.interp(tl[w], tav, tas), vz[w], alt[w]

fig, ax = plt.subplots(2, 1, figsize=(11, 8), sharex=True)
cols = ["#cc5555", "#2277bb", "#22aa55", "#aa55cc"]
maxt = 0
for k, (lb, ulg) in enumerate(runs):
    t, V, sink, alt = p9_series(ulg)
    ax[0].plot(t, sink, color=cols[k], lw=1.6, label="%s (peak %.1f)" % (lb, sink.max()))
    ax[1].plot(t, V, color=cols[k], lw=1.6, label="%s (end %.1f)" % (lb, V[-1]))
    maxt = max(maxt, t[-1])

# IDEAL: flat trapezoidal sink (snap ramps + constant), V flat at 38
ti = np.linspace(0, maxt, 400); R = 0.20; H = 148.0; wmax = H / (maxt * (1 - R)); fr = ti / maxt
shp = np.where(fr < R, P9(fr/R), np.where(fr > 1-R, P9((1-fr)/R), 1.0))
ax[0].plot(ti, wmax*shp, color="#000", ls="--", lw=1.4, label="IDEAL flat sink (peak %.1f)" % wmax)
ax[1].axhline(38, color="#000", ls="--", lw=1.4, label="IDEAL V = 38 (flat)")

ax[0].set_ylabel("sink-rate w [m/s]"); ax[0].set_title("P9 DESCEND: sink-rate(t) — measured (PEAKED snap-velocity) vs IDEAL (flat low-peak)", fontsize=10)
ax[0].legend(fontsize=8); ax[0].grid(alpha=.3)
ax[1].set_ylabel("V_TAS [m/s]"); ax[1].set_xlabel("t since P9 start [s]")
ax[1].set_title("V_TAS(t) — measured V RISES where sink peaks (mg*w spike); IDEAL stays 38", fontsize=10)
ax[1].axhline(38, color="#888", lw=0.4); ax[1].legend(fontsize=8); ax[1].grid(alpha=.3); ax[1].set_ylim(34, 46)
fig.suptitle("P9 sink-rate -> speed-rise: measured vs spec-ideal  (gravity power mg*V*sin(gamma) = mg*w)", fontsize=11)
plt.tight_layout(); plt.savefig(out, dpi=115); print("SAVED", out)
