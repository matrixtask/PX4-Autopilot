#!/usr/bin/env python3
# Choshi gust series, PEAK-TOTAL-30 variant: same model STRUCTURE, intensity rescaled so the TOTAL wind's
# 3-s max ~= 30 m/s (G=1.5 about a 20 m/s mean). Mean U=20 FROM North; Iu=0.15 -> sigma_u=3.0,
# sigma_v=0.78*sig_u=2.34, sigma_w=0.5*sig_u=1.50; SAME length scales (Lu130/Lv65/Lw25 -> tau=L/U);
# (1-cos) events 20->30 (3-s-mean 30: amplitude 10/0.818=12.2, inst peak 32.2) at t=200/500/800, max()-merged,
# clip [5.3, 32.3]. Fixed seed (same as the peak-45 series for comparability).
import numpy as np
np.random.seed(20260703)
DT = 0.1; T = 900.0; N = int(T/DT)
U0 = 20.0
SIG_U, SIG_V, SIG_W = 3.0, 2.34, 1.50
TAU_U, TAU_V, TAU_W = 130.0/U0, 65.0/U0, 25.0/U0
def ar1(sig, tau):
    a = np.exp(-DT/tau); b = sig*np.sqrt(1.0 - a*a)
    x = np.zeros(N)
    for k in range(1, N): x[k] = a*x[k-1] + b*np.random.randn()
    return x
u = ar1(SIG_U, TAU_U); v = ar1(SIG_V, TAU_V); w = ar1(SIG_W, TAU_W)
t = np.arange(N)*DT
env = np.zeros(N)
for tg in (200.0, 500.0, 800.0):
    m = (t >= tg) & (t < tg+6.0)
    env[m] = U0 + 12.2*0.5*(1.0 - np.cos(2*np.pi*(t[m]-tg)/6.0))
turb = U0 + u
speed_u = np.clip(np.where(env > 0, np.maximum(turb, env), turb), 5.3, 32.3)
wx = v; wy = -speed_u; wz = w
np.savetxt("/home/tasukunakai/mk7_choshi_wind_p30.csv", np.column_stack([t, wx, wy, wz, speed_u]),
           delimiter=",", header="t,wx_east,wy_north,wz_up,u_alongwind", comments="", fmt="%.3f")
k3 = int(3.0/DT)
m3 = np.convolve(speed_u, np.ones(k3)/k3, mode="valid")
print("Choshi PEAK-30 series: N=%d, seed fixed" % N)
print("  sigma_u meas=%.2f (tgt 3.0)  sigma_v=%.2f (2.34)  sigma_w=%.2f (1.50)" % (u.std(), v.std(), w.std()))
print("  mean u=%.1f  max inst=%.1f  min inst=%.1f" % (speed_u.mean(), speed_u.max(), speed_u.min()))
print("  MAX 3-s MEAN = %.1f m/s (target ~30 = G1.5 x 20)  G=%.2f" % (m3.max(), m3.max()/U0))
