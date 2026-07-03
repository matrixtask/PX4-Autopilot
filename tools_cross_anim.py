#!/usr/bin/env python3
# Attitude-visible 3D animation of the figure-8 CROSS trajectory (rpfC03). Aircraft body at real roll/pitch/yaw + comet trail.
# Reuses tools_attitude_anim.py; phase() adapted to the CROSS topology (45 NE out, CW202 + CCW202, course-freeze descent).
import sys, numpy as np, matplotlib
matplotlib.use("Agg")
import imageio_ffmpeg
matplotlib.rcParams["animation.ffmpeg_path"] = imageio_ffmpeg.get_ffmpeg_exe()
import matplotlib.pyplot as plt
from matplotlib import animation
from mpl_toolkits.mplot3d.art3d import Poly3DCollection
from pyulog import ULog

ulg, out = sys.argv[1], sys.argv[2]
u = ULog(ulg); D = {d.name: d.data for d in u.data_list}
t0 = D["vehicle_attitude"]["timestamp"][0]
def T(n): return (np.asarray(D[n]["timestamp"], float) - t0) / 1e6
lp = D["vehicle_local_position"]; tl = T("vehicle_local_position")
N = np.asarray(lp["x"]); E = np.asarray(lp["y"]); U = -np.asarray(lp["z"])
av = D["airspeed_validated"]; tav = T("airspeed_validated"); tas = np.asarray(av["true_airspeed_m_s"])
aq = D["vehicle_attitude"]; taq = T("vehicle_attitude")
qw, qx, qy, qz = [np.asarray(aq["q[%d]" % i]) for i in range(4)]
roll = np.degrees(np.arctan2(2*(qw*qx+qy*qz), 1-2*(qx*qx+qy*qy)))
pitch = np.degrees(np.arcsin(np.clip(2*(qw*qy-qz*qx), -1, 1)))
yaw = np.degrees(np.arctan2(2*(qw*qz+qx*qy), 1-2*(qy*qy+qz*qz))) % 360

fly = np.where(U > 0.5)[0]; i0, i1 = fly[0], len(U)-1
NF = 520; idx = np.linspace(i0, i1, NF).astype(int)
t = tl[idx]; Ef = E[idx]; Nf = N[idx]; Uf = U[idx]
Vf = np.interp(t, tav, tas); rf = np.interp(t, taq, roll); pf = np.interp(t, taq, pitch); yf = np.interp(t, taq, yaw)
qwi = np.interp(t, taq, qw); qxi = np.interp(t, taq, qx); qyi = np.interp(t, taq, qy); qzi = np.interp(t, taq, qz)

def phase(i):
    a, v, up, rl = Uf[i], Vf[i], (Uf[i]-Uf[max(0,i-3)]), rf[i]
    if a < 9:  return "P1/P2 Takeoff/Hover" if t[i] < t[NF//2] else "P12 Land (vertical)"
    if a < 150 and up > 0.25:  return "P4 Climb 1:4 (45 NE)"
    if a < 295 and up > 0.25:  return "P5 Climb X (V->C, 48)"
    if abs(rl) > 18 and a > 240:
        return "P7 Turn %s (infinity %s lobe)" % (("CW202","NE") if rl > 0 else ("CCW202","SW"))
    if a > 290 and v > 55:  return "P6 Cruise / cross leg (72, self-crossing)"
    if 150 < a < 300 and up < -0.25:  return "P9 Descend (38, course-freeze -> HOME)"
    if a < 152 and up < -0.25:  return "P10 Descend 1:4 (back-trans)"
    if a < 12 and v < 5:  return "P11 Hover&set"
    return "figure-8 CROSS-202"

SC = max(np.ptp(Ef), np.ptp(Nf)) * 0.045
fus = np.array([[-1.0,0,0],[1.4,0,0]])
wing = np.array([[0.3,-1.6,0],[0.3,1.6,0],[-0.3,1.6,0],[-0.3,-1.6,0]])
tailp = np.array([[-0.8,-0.6,0],[-0.8,0.6,0],[-1.1,0.6,0],[-1.1,-0.6,0]])
vfin = np.array([[-0.8,0,0],[-1.1,0,-0.7]])
def Rq(w,x,y,z):
    return np.array([[1-2*(y*y+z*z),2*(x*y-w*z),2*(x*z+w*y)],
                     [2*(x*y+w*z),1-2*(x*x+z*z),2*(y*z-w*x)],
                     [2*(x*z-w*y),2*(y*z+w*x),1-2*(x*x+y*y)]])
def to_enu(P): return np.column_stack([P[:,1], P[:,0], -P[:,2]])

fig = plt.figure(figsize=(10.5, 7.2)); fig.patch.set_facecolor("#0b0f17")
ax = fig.add_subplot(111, projection="3d"); ax.set_facecolor("#0b0f17")
pad = 0.06*max(np.ptp(Ef),np.ptp(Nf))
ax.set_xlim(Ef.min()-pad, Ef.max()+pad); ax.set_ylim(Nf.min()-pad, Nf.max()+pad); ax.set_zlim(0, Uf.max()*1.1)
ax.set_box_aspect((np.ptp(Ef)+2*pad, np.ptp(Nf)+2*pad, Uf.max()*1.1*5))
for a in (ax.xaxis,ax.yaxis,ax.zaxis): a.pane.set_facecolor((0,0,0,0)); a.pane.set_edgecolor((1,1,1,0.08))
ax.grid(False); ax.tick_params(colors="#5a6b85", labelsize=7)
ax.set_xlabel("East [m]",color="#88aacc",fontsize=8); ax.set_ylabel("North [m]",color="#88aacc",fontsize=8); ax.set_zlabel("alt [m]",color="#88aacc",fontsize=8)
ax.plot(Ef, Nf, Uf, color="#2b3a55", lw=0.9, alpha=0.6)
ax.scatter([0],[0],[0], color="gold", s=70, marker="*", depthshade=False)   # HOME
trail, = ax.plot([],[],[], color="#ff8c2a", lw=2.2)
wing_c = Poly3DCollection([], facecolor="#4aa3ff", edgecolor="#bfe0ff", alpha=0.9)
tail_c = Poly3DCollection([], facecolor="#4aa3ff", edgecolor="#bfe0ff", alpha=0.9)
ax.add_collection3d(wing_c); ax.add_collection3d(tail_c)
fus_l, = ax.plot([],[],[], color="#ffd24a", lw=2.6)
fin_l, = ax.plot([],[],[], color="#ffd24a", lw=2.0)
head, = ax.plot([],[],[], color="#ff5533", marker="o", ms=5)   # comet head
txt = ax.text2D(0.01, 0.97, "", transform=ax.transAxes, color="#cfe2ff", fontsize=9, va="top", family="monospace")
ax.set_title("Mk-7 figure-8 CROSS-202 (rpfC03): 45 NE out, CW202 + CCW202, self-crossing, HOME-aim descent", color="#cfe2ff", fontsize=9)

def body_world(i):
    Rm = Rq(qwi[i],qxi[i],qyi[i],qzi[i]); pos = np.array([Nf[i],Ef[i],-Uf[i]])
    def tf(P): return to_enu((Rm@(P*SC).T).T + pos)
    return tf(fus), tf(wing), tf(tailp), tf(vfin)

def upd(i):
    a = max(0,i-45); trail.set_data(Ef[a:i+1],Nf[a:i+1]); trail.set_3d_properties(Uf[a:i+1])
    f,w,tp,vf = body_world(i)
    fus_l.set_data(f[:,0],f[:,1]); fus_l.set_3d_properties(f[:,2])
    fin_l.set_data(vf[:,0],vf[:,1]); fin_l.set_3d_properties(vf[:,2])
    head.set_data([Ef[i]],[Nf[i]]); head.set_3d_properties([Uf[i]])
    wing_c.set_verts([w]); tail_c.set_verts([tp])
    ax.view_init(elev=22, azim=-60 + i*0.14)
    txt.set_text("%s\nt=%3.0fs  alt=%3.0f m  V=%2.0f m/s TAS\nroll=%+5.1f  pitch=%+5.1f  yaw=%5.1f deg" %
                 (phase(i), t[i]-t[0], Uf[i], Vf[i], rf[i], pf[i], yf[i]))
    return trail, fus_l, fin_l, head, wing_c, tail_c, txt

an = animation.FuncAnimation(fig, upd, frames=NF, interval=33, blit=False)
W = animation.FFMpegWriter(fps=30, codec="libx264", extra_args=["-crf","20","-pix_fmt","yuv420p","-movflags","+faststart"])
an.save(out, writer=W, dpi=120, savefig_kwargs={"facecolor": fig.get_facecolor()})
print("SAVED", out)
