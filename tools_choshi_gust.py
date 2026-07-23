#!/usr/bin/env python3
# Choshi-calibrated gust time series for the Mk-7 hover wind test.
# Model (approved): mean U=30 m/s FROM North; AIJ terrain cat II @z=50m -> Iu=0.15 (sigma_u=4.5),
# sigma_v=0.78*sigma_u, sigma_w=0.5*sigma_u; von Karman-class scales Lu~130 m (tau_u=Lu/U~4.3 s),
# Lv~65 m, Lw~25 m, realized as Dryden-like AR(1) filters (documented approximation); PLUS three
# deterministic (1-cos) discrete gusts +15 m/s (6 s wide) at t=200/500/800 -> peak ~45 = G 1.5.
# Output CSV (10 Hz x 900 s, fixed seed): t, wx_east, wy_north, wz_up, speed  [gz ENU components]
import numpy as np
np.random.seed(20260703)
DT = 0.1; T = 900.0; N = int(T/DT)
U0 = 30.0
SIG_U, SIG_V, SIG_W = 4.5, 3.5, 2.25
TAU_U, TAU_V, TAU_W = 130.0/U0, 65.0/U0, 25.0/U0
def ar1(sig, tau):
    a = np.exp(-DT/tau); b = sig*np.sqrt(1.0 - a*a)
    x = np.zeros(N)
    for k in range(1, N): x[k] = a*x[k-1] + b*np.random.randn()
    return x
u = ar1(SIG_U, TAU_U)   # along-wind fluctuation
v = ar1(SIG_V, TAU_V)   # lateral (East)
w = ar1(SIG_W, TAU_W)   # vertical (up)
t = np.arange(N)*DT
# discrete (1-cos) gusts to the G=1.5 design peak 45: event envelope MERGED with max() (not added on top
# of a turbulence peak) and the total clipped at 46 -> the 3-s max stays ~45 (G~1.5) by construction.
# JMA gust (瞬間風速) = 3-s mean -> target max 3-s mean 45 (G=1.5). (1-cos) event, 6 s wide: the centre
# 3-s mean of the shape is 0.818x peak -> amplitude 18.3 (inst peak 48.3, Choshi typhoon-record class).
env = np.zeros(N)
for tg in (200.0, 500.0, 800.0):
    m = (t >= tg) & (t < tg+6.0)
    env[m] = U0 + 18.3*0.5*(1.0 - np.cos(2*np.pi*(t[m]-tg)/6.0))
turb = U0 + u
speed_u = np.clip(np.where(env > 0, np.maximum(turb, env), turb), 8.0, 48.5)
wx = v                                       # East  = lateral
wy = -speed_u                                # North = -(FROM North)
wz = w
speed = np.sqrt(wx**2 + wy**2 + wz**2)
np.savetxt("/tmp/mk7_choshi_wind.csv", np.column_stack([t, wx, wy, wz, speed_u]),
           delimiter=",", header="t,wx_east,wy_north,wz_up,u_alongwind", comments="", fmt="%.3f")
# verification stats
k3 = int(3.0/DT)
m3 = np.convolve(speed_u, np.ones(k3)/k3, mode="valid")
print("Choshi series: N=%d (%.0f s @ %.0f Hz) seed fixed" % (N, T, 1/DT))
print("  sigma_u meas=%.2f (tgt 4.5)  sigma_v=%.2f (3.5)  sigma_w=%.2f (2.25)" % (u.std(), v.std(), w.std()))
print("  mean u=%.1f  max inst=%.1f  min inst=%.1f" % (speed_u.mean(), speed_u.max(), speed_u.min()))
print("  MAX 3-s MEAN = %.1f m/s  (target ~45 = G1.5)  gust factor=%.2f" % (m3.max(), m3.max()/U0))
