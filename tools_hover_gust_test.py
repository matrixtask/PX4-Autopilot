#!/usr/bin/env python3
# Mk-7 hover gust-resistance test: OFFBOARD vertical climb to 50 m, then position-hold (yaw=0 North=upwind)
# for --duration s while the Choshi wind plays. Pusher-assist (stock VT_FWD_THRUST_EN) carries the
# horizontal hold force so the body stays near-level. Prints peak XYZ deviations (PEAK rule).
import argparse, math, os, threading, time
from pymavlink import mavutil

ap = argparse.ArgumentParser()
ap.add_argument("--connect", default="udpin:0.0.0.0:14550")
ap.add_argument("--duration", type=float, default=90.0)
ap.add_argument("--alt", type=float, default=50.0)
a = ap.parse_args()

MASK_VEL_YAW = 0b0000100111000111   # use vel+yaw
MASK_POS_YAW = 0b0000100111111000   # use pos+yaw
def P9(x):
    x = max(0.0, min(1.0, x)); return x*x*x*x*x*(126 - 420*x + 540*x*x - 315*x**3 + 70*x**4)

m = mavutil.mavlink_connection(a.connect)
m.wait_heartbeat(); print("heartbeat", flush=True)
# GCS heartbeat thread: satisfies the "No connection to the GCS" preflight check (same as the mission script)
threading.Thread(target=lambda: [(m.mav.heartbeat_send(6, 8, 0, 0, 4), time.sleep(1.0)) for _ in iter(int, 1)], daemon=True).start()
INT_PARAMS = {"COM_RCL_EXCEPT", "NAV_RCL_ACT", "COM_OBL_RC_ACT", "VT_FWD_THRUST_EN"}
def set_param(name, val):
    # mission-proven: INT params MUST go as MAV_PARAM_TYPE_INT32 (REAL32 is silently rejected) + ACK-verify retry
    ptype = mavutil.mavlink.MAV_PARAM_TYPE_INT32 if name in INT_PARAMS else mavutil.mavlink.MAV_PARAM_TYPE_REAL32
    ok = False
    for _ in range(6):
        m.mav.param_set_send(m.target_system, m.target_component, name.encode(), float(val), ptype)
        pv = m.recv_match(type="PARAM_VALUE", blocking=True, timeout=1.5); t1 = time.time()
        while pv and pv.param_id != name and time.time()-t1 < 1.5:
            pv = m.recv_match(type="PARAM_VALUE", blocking=True, timeout=1.0)
        if pv and pv.param_id == name and abs(pv.param_value - float(val)) < 1e-3:
            ok = True; break
    print("  param %s = %s %s" % (name, val, "OK" if ok else "FAILED"), flush=True)
def cmd(c, *args):
    aa = list(args) + [0]*(7-len(args))
    m.mav.command_long_send(m.target_system, m.target_component, c, 0, *aa)

# failsafe-quiet + pusher-assist + tuning knobs (env-overridable)
for nm, dv in [("COM_RCL_EXCEPT",7),("NAV_RCL_ACT",0),("COM_OBL_RC_ACT",0),
               ("VT_FWD_THRUST_EN", float(os.environ.get("FWDEN","1"))),
               ("VT_FWD_THRUST_SC", float(os.environ.get("FWDSC","1.2"))),
               ("MPC_TILTMAX_AIR", float(os.environ.get("TILT","25"))),
               ("MPC_XY_VEL_MAX", float(os.environ.get("VELMAX","4"))), ("MPC_Z_VEL_MAX_UP", 4.0), ("MPC_Z_VEL_MAX_DN", 3.0)]:
    set_param(nm, dv)
for env, pn in [("XYP","MPC_XY_P"),("XYVP","MPC_XY_VEL_P_ACC"),("XYVD","MPC_XY_VEL_D_ACC"),
                ("XYVI","MPC_XY_VEL_I_ACC"),("ZP","MPC_Z_P"),("PITCHMIN","VT_PITCH_MIN"),
                ("SLEW","VT_PSHER_SLEW")]:
    if os.environ.get(env): set_param(pn, float(os.environ[env]))

g = None
while g is None or g.lat == 0:
    g = m.recv_match(type="GLOBAL_POSITION_INT", blocking=True, timeout=2)
print("got global pos", flush=True); time.sleep(2)
# ARM with retry + verification (fresh-world EKF/GPS-drift settle can reject the first attempts)
armed = False
for k in range(30):
    cmd(mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM, 1.0)
    time.sleep(2.0)
    hb = m.recv_match(type="HEARTBEAT", blocking=True, timeout=2)
    if hb and (hb.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED):
        armed = True; print("ARMED (attempt %d)" % (k+1), flush=True); break
if not armed:
    print("FAILED_ARM after 30 attempts", flush=True); raise SystemExit(1)

# P1-style OFFBOARD vertical climb to alt (calm; wind starts after the hold marker)
tv0 = time.time(); alt_now = 0.0
cmd(mavutil.mavlink.MAV_CMD_DO_SET_MODE, mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED, 6, 0)
while time.time()-tv0 < 60:
    wv = P9(min(1.0, (time.time()-tv0)/6.0))
    m.mav.set_position_target_global_int_send(0, m.target_system, m.target_component,
        mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT_INT, MASK_VEL_YAW,
        0, 0, 0.0, 0.0, 0.0, float(-3.0*wv), 0, 0, 0, 0.0, 0)   # yaw=0 rad = North (upwind)
    if time.time()-tv0 < 1.5:
        cmd(mavutil.mavlink.MAV_CMD_DO_SET_MODE, mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED, 6, 0)
    g2 = m.recv_match(type="GLOBAL_POSITION_INT", blocking=False)
    if g2: alt_now = g2.relative_alt/1000.0
    if alt_now >= a.alt - 0.5: break
    time.sleep(0.1)
if alt_now < a.alt - 3.0:
    print("FAILED_TAKEOFF alt=%.1f (target %.0f) — no wind marker" % (alt_now, a.alt), flush=True); raise SystemExit(1)
print("=> reached %.1f m, capturing hold point" % alt_now, flush=True)
g3 = m.recv_match(type="GLOBAL_POSITION_INT", blocking=True, timeout=2)
lat0, lon0 = g3.lat/1e7, g3.lon/1e7
t0 = time.time()
open("/tmp/mk7_wind_go", "w").write("%.3f" % t0)   # wind publisher trigger: series t=0 == hold start
print("=> HOLD start (marker written). duration=%.0f s" % a.duration, flush=True)

pkN = pkE = pkU = 0.0; last = 0.0
mlat = 111320.0; mlon = 111320.0*math.cos(math.radians(lat0))
FFK = float(os.environ.get("FFK", "0.0"))   # wind feedforward gain [m per (u^2/900-1)]: bias the hold point UPWIND
biasN = 0.0                                  # by the known gust force (anemometer-equivalent FF; wind from North)
while time.time()-t0 < a.duration:
    if FFK > 0.0:
        try:
            us = float(open("/tmp/mk7_wind_now").read().split()[0].split("=")[1])
            biasN = max(-6.0, min(6.0, FFK*(us*us/900.0 - 1.0)))
        except Exception: pass
    m.mav.set_position_target_global_int_send(0, m.target_system, m.target_component,
        mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT_INT, MASK_POS_YAW,
        int((lat0 + biasN/mlat)*1e7), int(lon0*1e7), float(a.alt), 0, 0, 0, 0, 0, 0, 0.0, 0)
    g4 = m.recv_match(type="GLOBAL_POSITION_INT", blocking=False)
    if g4 and g4.lat != 0:
        dN = (g4.lat/1e7-lat0)*mlat; dE = (g4.lon/1e7-lon0)*mlon; dU = g4.relative_alt/1000.0 - a.alt
        pkN = max(pkN, abs(dN)); pkE = max(pkE, abs(dE)); pkU = max(pkU, abs(dU))
        if time.time()-last > 5.0:
            last = time.time()
            wnow = "-"
            try: wnow = open("/tmp/mk7_wind_now").read().strip()
            except Exception: pass
            print("  t=%5.0f dN=%+6.2f dE=%+6.2f dU=%+6.2f | pk %5.2f %5.2f %5.2f | wind %s" %
                  (time.time()-t0, dN, dE, dU, pkN, pkE, pkU, wnow), flush=True)
    time.sleep(0.1)
print("RESULT hover peaks: |dN|=%.2f |dE|=%.2f |dU|=%.2f m (limit 2.0)" % (pkN, pkE, pkU), flush=True)
open("/tmp/mk7_wind_stop", "w").write("1")
print("DONE_HOVER", flush=True)
