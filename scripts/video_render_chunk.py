"""Renders submission-video frames [A, B) to /tmp/vidframes/%05d.png.

Scene plan (30 fps): title card, nominal mission (certified acquisition,
seeking, station keeping), disturbance card, 60 deg relay-rotation recovery,
results card. Chunked so each invocation fits constrained wall-time limits;
scripts/make_video.py documents the full pipeline and assembly command.
All content is honestly labeled numerical simulation (100 Hz dynamics,
20 Hz relay packets); no statistic in the paper comes from these traces.
"""
import csv, sys
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np

A, B = int(sys.argv[1]), int(sys.argv[2])
def load(p):
    r = list(csv.DictReader(open(p)))
    g = lambda k: np.array([float(x[k]) for x in r])
    return {k: g(k) for k in r[0].keys()}
nom = load("/tmp/viz_nominal.csv"); yaw = load("/tmp/viz_yawstep.csv")
MODES = {0: ("EXCITE","tab:orange"),1:("SEEK","tab:blue"),2:("MAINTAIN","tab:green")}
FPS = 30  # submission video requires at least 20 fps
frames = []
frames += [("title",k) for k in range(4*FPS)]
frames += [("nominal",k) for k in range(22*FPS)]
frames += [("cardA",k) for k in range(4*FPS)]
frames += [("yawstep",k) for k in range(26*FPS)]
frames += [("results",k) for k in range(7*FPS)]

fig = plt.figure(figsize=(12.8,7.2), dpi=75)
def title_card(text, sub):
    fig.clf(); ax = fig.add_axes([0,0,1,1]); ax.axis("off")
    ax.text(0.5,0.62,text,ha="center",va="center",fontsize=24)
    ax.text(0.5,0.38,sub,ha="center",va="center",fontsize=14,color="0.35")
def scene(kind,k):
    d = nom if kind=="nominal" else yaw
    secs = 22 if kind=="nominal" else 26
    i = min(int(k/(secs*FPS)*len(d["t"])), len(d["t"])-1)
    fig.clf()
    axT = fig.add_axes([0.06,0.12,0.52,0.76])
    axC = fig.add_axes([0.66,0.56,0.30,0.32])
    axD = fig.add_axes([0.66,0.12,0.30,0.32])
    fig.text(0.5,0.95,"Certified acquisition, seeking, station keeping" if kind=="nominal"
             else "Recovery after a 60 deg relay-frame disturbance at t = 8 s",
             ha="center",fontsize=14)
    fig.text(0.5,0.02,"Numerical simulation (100 Hz dynamics, 20 Hz relay packets); "
             "paper statistics come from the provenance-locked campaign.",
             ha="center",fontsize=8,color="0.4")
    qx,qy = d["qx"][:i+1], d["qy"][:i+1]
    axT.plot(d["drx"][:i+1], d["dry"][:i+1], color="0.6", lw=1, label="dead reckoning")
    axT.plot(qx,qy,color="tab:blue",lw=1.6,label="vehicle (true)")
    axT.plot(d["px"][0],d["py"][0],"r*",ms=16,label="hidden target")
    rx,ry,ryaw = d["rx"][0], d["ry"][0], d["relay_yaw"][i]
    axT.plot(rx,ry,"g^",ms=11,label="relay (pose unknown)")
    axT.annotate("",xy=(rx+1.6*np.cos(ryaw),ry+1.6*np.sin(ryaw)),xytext=(rx,ry),
                 arrowprops=dict(arrowstyle="->",color="g",lw=1.6))
    m,c = MODES[int(d["mode"][i])]
    axT.plot(qx[-1],qy[-1],"o",color=c,ms=9)
    axT.set_title(f"mode: {m}",color=c,fontsize=12)
    axT.axis("equal"); axT.grid(alpha=0.3); axT.legend(fontsize=8,loc="upper left")
    axT.set_xlabel("X [m]"); axT.set_ylabel("Y [m]")
    t = d["t"][:i+1]
    axC.plot(t,d["cert_hw_deg"][:i+1],color="tab:purple",lw=1.4)
    axC.axhline(10,color="k",lw=0.8,ls="--"); axC.set_ylim(0,30)
    axC.set_title("certificate halfwidth [deg]",fontsize=9); axC.grid(alpha=0.3)
    if kind=="yawstep":
        axC.axvline(8,color="r",lw=1,ls=":")
        axC.text(8.5,26,"relay rotated 60 deg",color="r",fontsize=8)
    axD.semilogy(t,np.maximum(d["dist"][:i+1],1e-2),color="tab:blue",lw=1.4)
    axD.set_ylim(0.01,30); axD.set_title("distance to hidden target [m]",fontsize=9)
    axD.set_xlabel("time [s]"); axD.grid(alpha=0.3,which="both")

for idx in range(A, min(B, len(frames))):
    kind,k = frames[idx]
    if kind=="title":
        title_card("GPS-Free Hidden-Target Seeking with a Calibrated\nYaw Certificate under Correlated Odometry",
                   "No GPS. Unknown-pose relay. Body-frame odometry only.\nAnonymous submission video (numerical simulation).")
    elif kind=="cardA":
        title_card("Disturbance: the relay is physically\nrotated by 60 deg mid-mission",
                   "Certified-window change detection must recognize\nthe new frame and recover.")
    elif kind=="results":
        title_card("Results (200 randomized trials per cell)",
                   "0.064 m median station error, matching a known-yaw oracle (0.065 m)\n"
                   "100% success through 50% dropout, 0.5 s delay, 20% outliers\n"
                   "95% certificate coverage: 94.8% realized\n"
                   "Dead reckoning drifts to 27 m; task error does not accumulate")
    else:
        scene(kind,k)
    fig.savefig(f"/tmp/vidframes/{idx:05d}.png")
print(f"rendered {A}..{min(B,len(frames))} of {len(frames)}")
