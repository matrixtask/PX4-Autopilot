#!/usr/bin/env python3
# Force-tracking verification (p30F3): does the CONTROL EFFORT (pusher throttle, pitch/tilt, lift thrust)
# follow the gust input in time (actively cancelling the external force)? Wind is reconstructed from the
# deterministic series (t_series=0 at hold+45 s ramp end). Outputs figure + correlation/lag + gust-event table.
import numpy as np, matplotlib
matplotlib.use("Agg"); import matplotlib.pyplot as plt
from pyulog import ULog
u = ULog("/home/tasukunakai/PX4-mk7-tune/build/px4_sitl_default/rootfs/log/2026-07-23/12_10_06.ulg")
D = {d.name: d.data for d in u.data_list}
t0 = D["vehicle_attitude"]["timestamp"][0]
def T(n): return (np.asarray(D[n]["timestamp"], float) - t0) / 1e6
lp = D["vehicle_local_position"]; tl = T("vehicle_local_position")
N = np.asarray(lp["x"]); E = np.asarray(lp["y"]); alt = -np.asarray(lp["z"])
ih = np.where(alt > 49.0)[0][0]; th = tl[ih]
ts = D["trajectory_setpoint"]; tts = T("trajectory_setpoint")
msp = (tts >= th+60) & (tts <= th+960)
spN = np.nanmean(np.asarray(ts["position[0]"])[msp]); spE = np.nanmean(np.asarray(ts["position[1]"])[msp])
mo = D["actuator_motors"]; tm = T("actuator_motors")
pu = np.asarray(mo["control[12]"])                                   # pusher
lift = np.mean([np.asarray(mo["control[%d]" % i]) for i in range(12)], axis=0)
aq = D["vehicle_attitude"]; ta = T("vehicle_attitude")
qw, qx, qy, qz = [np.asarray(aq["q[%d]" % i]) for i in range(4)]
pitch = np.degrees(np.arcsin(np.clip(2*(qw*qy - qz*qx), -1, 1)))
W = np.genfromtxt("/home/tasukunakai/mk7_choshi_wind_p30.csv", delimiter=",", names=True)
tw = W["t"] + th + 45.0; uw = W["u_alongwind"]                       # series t=0 == ramp end (hold+45)
g = np.arange(th+60, th+960, 0.1)
uu = np.interp(g, tw, uw)
pug = np.interp(g, tm[np.isfinite(pu)], pu[np.isfinite(pu)])
pig = np.interp(g, ta, pitch)
lfg = np.interp(g, tm, lift)
dNg = np.interp(g, tl, N) - spN
def xcorr(a, b, lags):
    a = a - a.mean(); b = b - b.mean()
    out = []
    for k in lags:
        out.append(np.corrcoef(a[max(0,k):len(a)+min(0,k)], b[max(0,-k):len(b)-max(0,k)])[0,1])
    return np.array(out)
lags = np.arange(0, 61)                                              # 0..6 s (control lags the wind)
cP = xcorr(uu, pug, lags); kP = int(np.argmax(cP))
cL = xcorr(uu, lfg, lags); kL = int(np.argmax(np.abs(cL)))
print("CORRELATION (steady 60-960 s, wind u -> control):")
print("  pusher:  r_max=%+.2f at lag=%.1f s" % (cP[kP], lags[kP]*0.1))
print("  liftthr: r=%+.2f at lag=%.1f s" % (cL[kL], lags[kL]*0.1))
print("\nDISCRETE GUSTS (20->30 m/s at series 200/500/800):")
for tg in (200.0, 500.0, 800.0):
    c = th + 45 + tg
    pre = (g > c-20) & (g < c-5); ev = (g > c) & (g < c+8)
    print("  t=%3.0fs: pusher %.2f->%.2f (+%.0f%%)  pitch %+.1f->%+.1f deg  |dN| max %.2f m" % (
        tg, pug[pre].mean(), pug[ev].max(), 100*(pug[ev].max()/max(pug[pre].mean(),1e-3)-1),
        pig[pre].mean(), pig[ev].min(), np.abs(dNg[ev]).max()))
fig, axs = plt.subplots(3, 1, figsize=(14, 9), sharex=True)
tt = g - th
axs[0].plot(tt, uu, lw=0.7, color="gray"); axs[0].set_ylabel("wind u [m/s]")
axs[0].set_title("Mk-7 hover force-tracking (p30F3, D=1.2): gust input vs control effort vs position\npusher r=%+.2f @lag %.1f s — the control ACTIVELY follows the gust force" % (cP[kP], lags[kP]*0.1), fontsize=10)
axs[1].plot(tt, pug, lw=0.7, color="#d62728", label="pusher throttle [-]")
ax1b = axs[1].twinx(); ax1b.plot(tt, pig, lw=0.6, color="#1f77b4", alpha=0.7, label="pitch [deg]")
axs[1].set_ylabel("pusher [-]", color="#d62728"); ax1b.set_ylabel("pitch [deg]", color="#1f77b4")
axs[2].plot(tt, dNg, lw=0.7, color="#2ca02c"); axs[2].axhspan(-2, 2, color="green", alpha=0.08)
axs[2].set_ylabel("dN (setpoint-ref) [m]"); axs[2].set_xlabel("time since hold start [s]")
for a in axs: a.grid(alpha=.3)
for tg in (245, 545, 845):
    for a in axs: a.axvline(tg, color="orange", lw=0.8, ls="--", alpha=0.7)
plt.tight_layout(); plt.savefig("/tmp/mk7_hover_gust_forcetrack.png", dpi=125)
print("\nSAVED /tmp/mk7_hover_gust_forcetrack.png")
