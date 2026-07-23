#!/usr/bin/env python3
# Choshi wind playback: waits for /tmp/mk7_wind_go, ramps 0->30 over --ramp s, then plays the CSV series
# via `gz topic` publishes to /world/<world>/wind (gz.msgs.Wind). Stateless velocity updates (WindEffects
# computes the force each step from the CURRENT wind -> no ApplyLinkWrench-style accumulation).
# Logs (wall_t, series_t, wx, wy, wz, u) to /tmp/mk7_wind_played.csv and the current u to /tmp/mk7_wind_now.
import argparse, os, subprocess, time
ap = argparse.ArgumentParser()
ap.add_argument("--csv", default="/tmp/mk7_choshi_wind.csv")
ap.add_argument("--world", default="teTra_wind_choshi")
ap.add_argument("--rate", type=float, default=2.5)
ap.add_argument("--ramp", type=float, default=20.0)
ap.add_argument("--umean", type=float, default=30.0)   # mean wind for the ramp target (peak-30 variant uses 20)
a = ap.parse_args()
rows = []
with open(a.csv) as f:
    next(f)
    for ln in f:
        p = ln.strip().split(",")
        rows.append((float(p[0]), float(p[1]), float(p[2]), float(p[3]), float(p[4])))
TOPIC = "/world/%s/wind" % a.world
def pub(wx, wy, wz):
    # TRUE wind (no squared hack): gz-sim8 LiftDrag reads the Wind component (WindTag confirmed in the .so),
    # so the WING/tail see the physical flow aerodynamically; the WindEffects body force (fuselage+stopped
    # rotors only, CdS_fus~1.0 m^2) is calibrated linearly via force_approximation_scaling_factor=0.0118.
    msg = "linear_velocity: {x: %.2f, y: %.2f, z: %.2f}, enable_wind: true" % (wx, wy, wz)
    subprocess.run(["gz", "topic", "-t", TOPIC, "-m", "gz.msgs.Wind", "-p", msg],
                   stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL, timeout=5)
print("waiting for /tmp/mk7_wind_go (hold start)...", flush=True)
t_wait = time.time()
while not os.path.exists("/tmp/mk7_wind_go"):
    if time.time()-t_wait > 400: print("TIMEOUT waiting for marker"); raise SystemExit(1)
    time.sleep(0.5)
try: os.remove("/tmp/mk7_wind_stop")
except Exception: pass
log = open("/tmp/mk7_wind_played.csv", "w"); log.write("wall_t,series_t,wx,wy,wz,u\n")
t0 = time.time(); dtp = 1.0/a.rate
print("RAMP 0->%.0f over %.0f s, then series playback @%.1f Hz" % (a.umean, a.ramp, a.rate), flush=True)
def P9(x):
    x = max(0.0, min(1.0, x)); return x*x*x*x*x*(126 - 420*x + 540*x*x - 315*x**3 + 70*x**4)
while time.time()-t0 < a.ramp:
    if os.path.exists("/tmp/mk7_wind_stop"): break
    u = a.umean*P9((time.time()-t0)/a.ramp)
    pub(0.0, -u, 0.0)
    log.write("%.2f,%.2f,0.00,%.2f,0.00,%.2f\n" % (time.time(), (time.time()-t0)-a.ramp, -u, u)); log.flush()
    open("/tmp/mk7_wind_now", "w").write("u=%.1f (ramp)" % u)
    time.sleep(dtp)
ts0 = time.time()
print("SERIES start", flush=True)
while True:
    if os.path.exists("/tmp/mk7_wind_stop"): print("stop marker -> end"); break
    st = time.time()-ts0
    k = int(st*10.0)
    if k >= len(rows): print("series complete (%.0f s)" % st); break
    _, wx, wy, wz, u = rows[k]
    pub(wx, wy, wz)
    log.write("%.2f,%.2f,%.2f,%.2f,%.2f,%.2f\n" % (time.time(), st, wx, wy, wz, u)); log.flush()
    open("/tmp/mk7_wind_now", "w").write("u=%.1f gustE=%.1f w=%.1f" % (u, wx, wz))
    time.sleep(dtp)
pub(0.0, 0.0, 0.0)
print("DONE_WIND", flush=True)
