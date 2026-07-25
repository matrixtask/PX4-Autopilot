#!/usr/bin/env python3
# Figure-8 CROSS prediction: out leg NORTH(0deg), OPPOSITE-rotation 270 lobes -> self-crossing infinity that returns to HOME.
# Tunes the cross/back leg lengths to (a) self-cross and (b) close near HOME, given the measured climb/descent footprints.
import numpy as np, matplotlib
matplotlib.use("Agg"); import matplotlib.pyplot as plt
R=72.0**2/(9.81*np.tan(np.radians(45)))      # 528 m
CLIMB=3119.0                                   # measured North out-leg footprint (climb 1:4->152->300)
def cruise(p,hdg,L,n=40):
    h=np.radians(hdg); return np.c_[p[0]+np.linspace(0,L,n)*np.sin(h), p[1]+np.linspace(0,L,n)*np.cos(h)]
def arc(p,hdg0,turn,R,n=80):                   # turn>0=CW(right), turn<0=CCW(left)
    h0=np.radians(hdg0); s=np.sign(turn); cx=p[0]+s*R*np.cos(h0); cy=p[1]-s*R*np.sin(h0)
    a0=np.arctan2(p[1]-cy,p[0]-cx); aa=a0 - s*np.radians(abs(turn))*np.linspace(0,1,n)
    return np.c_[cx+R*np.cos(aa), cy+R*np.sin(aa)]
def hdg_of(seg):
    d=seg[-1]-seg[-2]; return np.degrees(np.arctan2(d[0],d[1]))%360
def seg_cross(A,B):                            # do polylines A,B intersect (excluding shared endpoints)?
    for i in range(len(A)-1):
        for j in range(len(B)-1):
            p,r=A[i],A[i+1]-A[i]; q,s=B[j],B[j+1]-B[j]
            rxs=r[0]*s[1]-r[1]*s[0]
            if abs(rxs)<1e-9: continue
            t=((q[0]-p[0])*s[1]-(q[1]-p[1])*s[0])/rxs; uu=((q[0]-p[0])*r[1]-(q[1]-p[1])*r[0])/rxs
            if 0.02<t<0.98 and 0.02<uu<0.98: return p+t*r
    return None
HOME=np.array([0.0,0.0]); OUT_HDG=45.0                       # [v2] out leg = 45deg NE (was 0 North)
def build(LO, T1dir, LC, T2dir, t1=270.0, t2=270.0):
    S=[]
    s=cruise(HOME,OUT_HDG,LO); S.append(s)                   # out NE 45 (climb)
    s=arc(S[-1][-1],hdg_of(S[-1]),t1*T1dir,R); S.append(s)   # lobe-1 turn
    s=cruise(S[-1][-1],hdg_of(S[-1]),LC); S.append(s)        # cross leg
    s=arc(S[-1][-1],hdg_of(S[-1]),t2*T2dir,R); S.append(s)   # lobe-2 turn (opposite)
    p=S[-1][-1]; hh=np.degrees(np.arctan2(-p[0],-p[1]))%360; LB=np.hypot(*p)  # back leg AIMED straight at HOME
    s=cruise(p,hh,LB); S.append(s)
    return S, LB
def first_cross(S):
    for a,b in [(0,2),(0,4),(2,4),(1,3),(1,4)]:
        x=seg_cross(S[a],S[b])
        if x is not None: return x
    return None
# back leg aimed at HOME -> closure ~0 by construction; search opposite-rotation (CW then CCW) configs that SELF-CROSS,
# pick the most infinity-like (cross point nearest the figure mid-line, balanced lobes, modest back-leg length).
best=None
for d1,d2 in [(+1,-1)]:
    for tt in range(200,246,2):                              # find the MINIMUM turn angle (>=200, closest to user 200) that self-crosses
        t1=t2=tt
        for LC in np.arange(200, 5200, 30):
            S,LB=build(CLIMB,d1,LC,d2,t1,t2)
            if LB>7500 or LB<150: continue
            X=first_cross(S)
            allp=np.vstack(S); extN=allp[:,1].max()-allp[:,1].min(); extE=allp[:,0].max()-allp[:,0].min()
            score=(0 if X is not None else 90000)+(tt-200)*200+LB*0.02   # PREFER cross, then the LOWEST turn angle, then compact
            if best is None or score<best[0]: best=(score,t1,t2,LC,LB,X,extN,extE)
score,t1,t2,LC,LB,Xpt,extN,extE=best
S,_=build(CLIMB,+1,LC,-1,t1,t2); end=S[-1][-1]; cl=np.hypot(*end)
allp=np.vstack(S); extN=allp[:,1].max()-allp[:,1].min(); extE=allp[:,0].max()-allp[:,0].min()
def first_cross(S):
    for a,b in [(0,2),(0,4),(2,4)]:
        x=seg_cross(S[a],S[b])
        if x is not None: return x
    return None
Xpt=first_cross(S)
print("BEST fig-8 cross (out NORTH): Turn1 CW%d + Turn2 CCW%d, LC=%.0f, back-leg(->HOME)=%.0f -> closure=%.0f m"%(t1,t2,LC,LB,cl))
print("self-cross: %s; extent N-S=%.0f E-W=%.0f"%(("(E,N)=(%.0f,%.0f)"%(Xpt[0],Xpt[1])) if Xpt is not None else "NONE (200deg too shallow)",extN,extE))
labels=["out NE 45 (climb 3.1km)","Turn1 CW%d"%t1,"cross leg","Turn2 CCW%d (opp)"%t2,"back leg (descent)->HOME"]
cols=["#2ca02c","#d62728","#1f77b4","#e377c2","#8c564b"]
fig,ax=plt.subplots(figsize=(9,11))
for s,nm,c in zip(S,labels,cols): ax.plot(s[:,0],s[:,1],color=c,lw=2.6,label=nm)
ax.plot(0,0,"*",color="gold",ms=26,mec="k",zorder=6,label="HOME (takeoff+land)")
ax.plot(end[0],end[1],"X",color="red",ms=14,mec="k",zorder=6,label="end (closure %.0f m)"%cl)
if Xpt is not None: ax.plot(Xpt[0],Xpt[1],"o",color="black",ms=12,mfc="none",mew=2,zorder=6,label="self-CROSS")
ax.annotate("",xy=(354,354),xytext=(0,0),arrowprops=dict(arrowstyle="->",lw=2,color="gray"))
ax.set_aspect("equal"); ax.grid(alpha=.3); ax.set_xlabel("East [m]"); ax.set_ylabel("North [m]")
ax.set_title("Mk-7 FIGURE-8 CROSS v2: out 45deg NE, Turn1 CW%d + Turn2 CCW%d (opposite) -> self-crossing infinity\nN-S=%.1fkm E-W=%.1fkm; self-cross marked; HOME-return closure %.0f m"%(t1,t2,extN/1000,extE/1000,cl),fontsize=9.5)
ax.legend(loc="upper right",fontsize=8)
plt.tight_layout(); plt.savefig("/tmp/mk7_fig8cross_predict_v3.png",dpi=125); print("SAVED /tmp/mk7_fig8cross_predict_v3.png")
