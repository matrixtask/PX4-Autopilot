#!/usr/bin/env python3
# Dutch-roll mode ID: FFT/PSD of roll & yaw (+beta) in the FW phases P5/P8/P9, roll-yaw coherence & phase, damping zeta.
# usage: tools_dutchroll.py <ulog> <out_png>
import sys, numpy as np, matplotlib
matplotlib.use("Agg"); import matplotlib.pyplot as plt
from pyulog import ULog
try:
    from scipy.signal import welch, csd, coherence, detrend
    HAVE_SCIPY = True
except Exception:
    HAVE_SCIPY = False
ulg, outpng = sys.argv[1], sys.argv[2]
u = ULog(ulg); D = {d.name: d.data for d in u.data_list}
t0 = D["vehicle_attitude"]["timestamp"][0]
def T(n): return (np.asarray(D[n]["timestamp"], float) - t0) / 1e6
aqd = D["vehicle_attitude"]; taq = T("vehicle_attitude"); qa = [np.asarray(aqd["q[%d]" % i]) for i in range(4)]
n = min(len(taq), *[len(x) for x in qa]); taq = taq[:n]; qw, qx, qy, qz = [x[:n] for x in qa]
roll = np.degrees(np.arctan2(2*(qw*qx+qy*qz), 1-2*(qx*qx+qy*qy)))
yaw = np.degrees(np.unwrap(np.arctan2(2*(qw*qz+qx*qy), 1-2*(qy*qy+qz*qz))))
lp = D["vehicle_local_position"]; tl = T("vehicle_local_position")
alt = -np.asarray(lp["z"]); vN = np.asarray(lp["vx"]); vE = np.asarray(lp["vy"]); vD = np.asarray(lp["vz"]); V = np.hypot(vN, vE)
yawh = np.degrees(np.asarray(lp["heading"])) % 360; rollf = np.interp(tl, taq, roll)
# sideslip beta = asin(v_body_y / |v|), body velocity = R_w2b @ v_ned
qwl, qxl, qyl, qzl = [np.interp(tl, taq, x) for x in (qw, qx, qy, qz)]
def w2b_y(vn, ve, vd, w, x, y, z):  # body-Y component of world velocity
    return 2*(x*y+w*z)*vn + (1-2*(x*x+z*z))*ve + 2*(y*z-w*x)*vd
vby = w2b_y(vN, vE, vD, qwl, qxl, qyl, qzl); Vtot = np.maximum(np.sqrt(vN*vN+vE*vE+vD*vD), 1.0)
beta = np.degrees(np.arcsin(np.clip(vby/Vtot, -1, 1)))
def first(c, a):
    ix = np.where(c & (tl > a))[0]; return tl[ix[0]] if len(ix) else a
t_lift = tl[np.where(alt > 0.5)[0][0]]; t1 = first(alt > 9.3, t_lift); t2 = t1+8.0
t3 = first(np.abs((yawh-90+540) % 360-180) < 2.0, t2); t4 = first(alt > 151, t3); t5 = first(alt > 298, t4)
t6 = first(np.abs(rollf) > 12, t5+3); t7 = first((np.abs((yawh-270+540) % 360-180) < 12) & (np.abs(rollf) < 8), t6+8)
t8 = first(V < 43, t7+3); t9 = first(alt < 153, t8+3)
# FW oscillation windows (steady middle, avoid entry/exit transients)
WIN = {"P5 climb": (t4+3, t5-1), "P8 brake": (t7+4, t8-2), "P9 descend": (t8+4, t9-3)}
FS = 50.0
def resamp(t, y, a, b):
    g = np.arange(a, b, 1.0/FS); return g, np.interp(g, t, y)
def psd(y):
    if HAVE_SCIPY:
        f, p = welch(detrend(y), fs=FS, nperseg=min(len(y), int(FS*16)))
        return f, p
    yd = y - np.convolve(y, np.ones(int(FS*2))/int(FS*2), "same")
    Y = np.fft.rfft(yd*np.hanning(len(yd))); f = np.fft.rfftfreq(len(yd), 1.0/FS)
    return f, (np.abs(Y)**2)
def peakf(f, p, lo=0.15, hi=2.0):
    m = (f >= lo) & (f <= hi); i = np.argmax(p[m]); return f[m][i], p[m][i], m, i
def zeta_from_psd(f, p, fpk):
    m = (f > fpk*0.3) & (f < fpk*3); fm, pm = f[m], p[m]; ip = np.argmin(np.abs(fm-fpk)); ppk = pm[ip]
    half = ppk/2.0
    lo = fm[:ip][pm[:ip] <= half]; hi = fm[ip:][pm[ip:] <= half]
    if len(lo) and len(hi):
        bw = hi[0]-lo[-1]; return bw/(2*fpk)  # zeta ~ (df_-3dB)/(2 fn)
    return float("nan")
fig, ax = plt.subplots(3, 2, figsize=(14, 11))
print("phase        roll_fpk[Hz] yaw_fpk[Hz]  coh(roll,yaw)@fpk  phase(roll-yaw)[deg]  zeta_roll  beta_fpk[Hz]")
for r, (nm, (a, b)) in enumerate(WIN.items()):
    g, rr = resamp(taq, roll, a, b); _, yy = resamp(taq, yaw, a, b); _, bb = resamp(tl, beta, a, b)
    rr_d = rr - np.polyval(np.polyfit(g, rr, 2), g); yy_d = yy - np.polyval(np.polyfit(g, yy, 2), g); bb_d = bb - np.polyval(np.polyfit(g, bb, 2), g)
    fr, pr = psd(rr_d); fy, py = psd(yy_d); fb, pb = psd(bb_d)
    rfpk, _, _, _ = peakf(fr, pr); yfpk, _, _, _ = peakf(fy, py); bfpk, _, _, _ = peakf(fb, pb)
    # coherence + phase at roll peak
    if HAVE_SCIPY:
        fc, Cxy = coherence(rr_d, yy_d, fs=FS, nperseg=min(len(rr_d), int(FS*16)))
        fcs, Pxy = csd(rr_d, yy_d, fs=FS, nperseg=min(len(rr_d), int(FS*16)))
        ic = np.argmin(np.abs(fc-rfpk)); coh = Cxy[ic]; ph = np.degrees(np.angle(Pxy[np.argmin(np.abs(fcs-rfpk))]))
    else:
        coh = float("nan"); ph = float("nan")
    z = zeta_from_psd(fr, pr, rfpk)
    print("  %-11s  %8.3f    %8.3f      %7.2f           %8.0f            %6.3f      %6.3f" % (nm, rfpk, yfpk, coh, ph, z, bfpk))
    ax[r, 0].plot(g-a, rr_d, color="#1f77b4", lw=0.8, label="roll"); ax[r, 0].plot(g-a, yy_d, color="#9467bd", lw=0.8, label="yaw")
    ax[r, 0].plot(g-a, bb_d, color="#2ca02c", lw=0.6, alpha=0.6, label="beta")
    ax[r, 0].set_title("%s  detrended roll/yaw/beta (t %.0f-%.0f s)" % (nm, a, b), fontsize=9); ax[r, 0].set_ylabel("deg"); ax[r, 0].legend(fontsize=7); ax[r, 0].grid(alpha=.3)
    ax[r, 1].semilogy(fr, pr, color="#1f77b4", label="roll (fpk %.2f)" % rfpk); ax[r, 1].semilogy(fy, py, color="#9467bd", label="yaw (fpk %.2f)" % yfpk)
    ax[r, 1].axvline(rfpk, color="#cc3333", ls="--", lw=1); ax[r, 1].set_xlim(0, 2.0); ax[r, 1].set_title("%s PSD (zeta_roll~%.3f, coh~%.2f)" % (nm, z, coh), fontsize=9)
    ax[r, 1].set_xlabel("freq [Hz]"); ax[r, 1].legend(fontsize=7); ax[r, 1].grid(alpha=.3)
fig.suptitle("Mk-7 Dutch-roll mode ID — roll/yaw/beta FFT, coherence & damping (%s)" % ulg.split("/")[-1], fontsize=11)
plt.tight_layout(); plt.savefig(outpng, dpi=120); print("\nSAVED", outpng, "| scipy=%s" % HAVE_SCIPY)
