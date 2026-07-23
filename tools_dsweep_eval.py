#!/usr/bin/env python3
# D-gain sweep evaluation: for one run's ulog, compute (steady 60-240 s window, 250 Hz):
#   - horizontal deviation PEAKs and detrended oscillation (peak-to-peak about a 10-s moving mean)
#   - ground-speed response phase LAG vs the wind input (cross-correlation of vN vs -u_wind fluctuations)
import numpy as np, sys
from pyulog import ULog
ULG = sys.argv[1]; TAG = sys.argv[2] if len(sys.argv) > 2 else "run"
T0 = float(open("/tmp/mk7_wind_go").read().strip())
u = ULog(ULG); D = {d.name: d.data for d in u.data_list}
t0 = D["vehicle_attitude"]["timestamp"][0]
lp = D["vehicle_local_position"]; tl = (np.asarray(lp["timestamp"], float) - t0) / 1e6
N = np.asarray(lp["x"]); E = np.asarray(lp["y"]); alt = -np.asarray(lp["z"])
vN = np.asarray(lp["vx"])
ih = np.where(alt > 49.0)[0][0]; th = tl[ih]
tt = tl - th
m = (tt >= 60.0) & (tt <= 240.0)
dN = N - N[ih]; dE = E - E[ih]
def osc(s, t):
    # detrended peak-to-peak: remove a 10-s moving mean -> the oscillation about the drift
    dt = np.median(np.diff(t)); w = max(5, int(10.0/dt))
    tr = np.convolve(s, np.ones(w)/w, mode="same"); r = (s - tr)[w:-w]
    return r.max() - r.min(), r.std()
oN, sN = osc(dN[m], tt[m]); oE, sE = osc(dE[m], tt[m])
# phase lag: resample vN & wind u on a 10 Hz grid, correlate fluctuations
W = np.genfromtxt("/tmp/mk7_wind_played.csv", delimiter=",", names=True)
tw = W["wall_t"] - T0; uw = W["u"]
g = np.arange(60.0, 240.0, 0.1)
a = np.interp(g, tt, vN); b = np.interp(g, tw, uw)
a = a - a.mean(); b = b - b.mean()
lags = np.arange(-100, 101)   # +-10 s
cc = [np.corrcoef(a[max(0, k):len(a)+min(0, k)], b[max(0, -k):len(b)-max(0, k)])[0, 1] for k in lags]
k = int(np.argmax(np.abs(np.array(cc))))
print("%s: PEAK |dN|=%.2f |dE|=%.2f | OSC(p-p detrended) N=%.2f E=%.2f (std %.2f/%.2f) | vN-vs-wind lag=%.1f s (r=%.2f)"
      % (TAG, np.abs(dN[m]).max(), np.abs(dE[m]).max(), oN, oE, sN, sE, lags[k]*0.1, cc[k]))
