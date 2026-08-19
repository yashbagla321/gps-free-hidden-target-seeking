#!/usr/bin/env python3
"""Assembles the Gazebo-only illustrative reel from captured camera clips.

Inputs (produced by ros2_ws/src/gps_free_seeking_gz/scripts/
capture_camera_video.py against the video-capture twin world): for each
scenario in SCENARIOS, a camera clip "<scenario>.mp4", a per-frame
sim-time log "<scenario>.frames.csv", and the evaluator's ground-truth
telemetry "<scenario>.csv" (same schema as the citable Gazebo runs).

The raw camera feed periodically repeats a stale rendered frame for a
run of ticks under WSL2's software-rendering load (confirmed by ground
truth continuing to advance smoothly underneath); this script drops
exact-duplicate consecutive frames, then maps the surviving unique
frames onto a fixed-length viewing window per scenario using the same
proportional-index compression the offline submission video already
uses (scripts/video_render_chunk.py), so motion reads smoothly instead
of stuttering. This is a presentation choice, not a data fabrication:
every displayed frame is a real captured image at its real recorded
sim-time; only the frozen repeats are skipped.

Each scene pairs the camera view with two live telemetry panels pulled
from the evaluator CSV: distance to the hidden target (semilog, with the
success-reach threshold marked) and odometry position drift (raw
wheel-odometry error vs. ground truth, i.e. what dead reckoning alone
would accumulate). The camera panel's title also reports the current
EXCITE/SEEK/MAINTAIN supervisor mode, color-matched to the same
convention used by scripts/video_render_chunk.py.

Usage (run inside the WSL/ROS environment that has cv2 + matplotlib):
    python3 scripts/make_gazebo_video.py --all
Then assemble with ffmpeg (see OUT_FPS/FRAMEDIR below), or pass --all
which shells out to ffmpeg directly.
"""
import argparse
import csv
import os
import subprocess
import sys

import cv2
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np

FPS = 30
FRAMEDIR = "/tmp/gzvidframes"
IN_DIR = "/tmp/gfs_video"
OUT = "docs/campaign2027_submission/video/gazebo_only.mp4"

MODES = {0: ("EXCITE", "tab:orange"), 1: ("SEEK", "tab:blue"),
          2: ("MAINTAIN", "tab:green")}

SCENARIOS = [
    {
        "name": "nominal",
        "view_s": 15,
        "title": "Physics-based ROS 2 / Gazebo validation: nominal mission",
        "caption": "Differential-drive vehicle, wheel odometry from the "
                   "simulator's own drive-train integration, no comms "
                   "degradation.",
        "yaw_step_time": None,
    },
    {
        "name": "stress",
        "view_s": 17,
        "title": "Combined communication stress",
        "caption": "30% packet dropout, 0.2 s fixed delay, 0.1 s jitter, "
                   "5% gross outliers on the relay channel -- applied "
                   "simultaneously.",
        "yaw_step_time": None,
    },
    {
        "name": "disturbance_transit",
        "view_s": 21,
        "title": "60 deg relay-frame rotation mid-transit",
        "caption": "Relay physically rotated 60 deg at t=6.5 s, four to "
                   "five meters from the target; supervisor must detect "
                   "the new frame and recover.",
        "yaw_step_time": 6.5,
    },
]


def load_telemetry(name):
    path = os.path.join(IN_DIR, f"{name}.csv")
    rows = list(csv.DictReader(open(path)))
    g = lambda k: np.array([float(x[k]) for x in rows])
    return {k: g(k) for k in rows[0].keys()}


def load_unique_frames(name):
    """Returns a list of (sim_t, bgr_frame), deduplicating exact repeats."""
    frames_csv = os.path.join(IN_DIR, f"{name}.frames.csv")
    sim_t = {}
    for r in csv.DictReader(open(frames_csv)):
        sim_t[int(r["frame_idx"])] = float(r["sim_t"])
    cap = cv2.VideoCapture(os.path.join(IN_DIR, f"{name}.mp4"))
    total = int(cap.get(cv2.CAP_PROP_FRAME_COUNT))
    out = []
    prev = None
    for idx in range(total):
        ret, frame = cap.read()
        if not ret:
            break
        if prev is not None and np.array_equal(frame, prev):
            continue
        out.append((sim_t.get(idx, idx / 30.0), frame))
        prev = frame
    cap.release()
    return out


def nearest_row(t_array, t):
    i = int(np.searchsorted(t_array, t))
    return min(max(i, 0), len(t_array) - 1)


fig = plt.figure(figsize=(12.8, 7.2), dpi=75)


def title_card(text, sub):
    fig.clf()
    ax = fig.add_axes([0, 0, 1, 1])
    ax.axis("off")
    ax.text(0.5, 0.62, text, ha="center", va="center", fontsize=22, wrap=True)
    ax.text(0.5, 0.36, sub, ha="center", va="center", fontsize=13,
             color="0.35", wrap=True)


def scene_frame(scenario, cam_frame, t, telem):
    fig.clf()
    axCam = fig.add_axes([0.04, 0.08, 0.56, 0.68])
    axD = fig.add_axes([0.66, 0.42, 0.30, 0.30])
    axO = fig.add_axes([0.66, 0.08, 0.30, 0.24])

    fig.text(0.5, 0.965, scenario["title"], ha="center", fontsize=15)
    fig.text(0.5, 0.885, scenario["caption"], ha="center", va="top",
             fontsize=9.5, color="0.3", wrap=True)
    fig.text(0.5, 0.02, "Physics-based ROS 2/Gazebo validation (illustrative "
             "capture); paper statistics come from the provenance-locked "
             "30-launch campaign, not this clip.", ha="center", fontsize=8,
             color="0.4")

    ti = telem["t"]
    i = nearest_row(ti, t)
    tt = ti[:i + 1]
    mode_name, mode_color = MODES.get(int(telem["mode"][i]), ("?", "0.3"))

    rgb = cv2.cvtColor(cam_frame, cv2.COLOR_BGR2RGB)
    axCam.imshow(rgb)
    axCam.axis("off")
    axCam.set_title(f"mode: {mode_name}   t = {t:0.1f} s", fontsize=12,
                    color=mode_color, fontweight="bold")
    axD.semilogy(tt, np.maximum(telem["dist"][:i + 1], 1e-2),
                 color="tab:blue", lw=1.4)
    axD.axhline(0.25, color="k", lw=0.8, ls="--")
    axD.set_ylim(0.01, 15)
    axD.set_title("distance to hidden target [m]", fontsize=9)
    axD.grid(alpha=0.3, which="both")
    if scenario["yaw_step_time"] is not None:
        yst = scenario["yaw_step_time"]
        if t >= yst:
            axD.axvline(yst, color="r", lw=1, ls=":")

    axO.plot(tt, telem["odom_pos_err"][:i + 1], color="tab:orange", lw=1.4)
    axO.set_title("odometry position drift [m]", fontsize=9)
    axO.set_xlabel("time [s]")
    axO.grid(alpha=0.3)
    if scenario["yaw_step_time"] is not None:
        yst = scenario["yaw_step_time"]
        if t >= yst:
            axO.axvline(yst, color="r", lw=1, ls=":")
            axO.text(yst + 0.3, axO.get_ylim()[1] * 0.85,
                     "relay rotated 60 deg", color="r", fontsize=8)


def render_all():
    os.makedirs(FRAMEDIR, exist_ok=True)
    idx = 0

    title_card("GPS-Free Hidden-Target Seeking with a Calibrated\n"
               "Yaw Certificate under Correlated Odometry",
               "Physics-based ROS 2/Gazebo validation (illustrative "
               "capture, not the citable campaign)")
    for _ in range(4 * FPS):
        fig.savefig(f"{FRAMEDIR}/{idx:05d}.png")
        idx += 1
    print(f"rendered title card, idx={idx}")

    for scenario in SCENARIOS:
        name = scenario["name"]
        print(f"loading {name}...")
        unique = load_unique_frames(name)
        telem = load_telemetry(name)
        print(f"  {len(unique)} unique frames, "
              f"{len(telem['t'])} telemetry rows")

        title_card(scenario["title"], scenario["caption"])
        for _ in range(3 * FPS):
            fig.savefig(f"{FRAMEDIR}/{idx:05d}.png")
            idx += 1

        n_out = scenario["view_s"] * FPS
        for k in range(n_out):
            j = min(int(k / n_out * len(unique)), len(unique) - 1)
            t, cam_frame = unique[j]
            scene_frame(scenario, cam_frame, t, telem)
            fig.savefig(f"{FRAMEDIR}/{idx:05d}.png")
            idx += 1
        print(f"  rendered scene, idx={idx}")

    print(f"total frames rendered: {idx}")
    return idx


def assemble():
    os.makedirs(os.path.dirname(OUT), exist_ok=True)
    ffmpeg = os.path.expanduser("~/.local/bin/ffmpeg")
    if not os.path.exists(ffmpeg):
        ffmpeg = "ffmpeg"
    cmd = [ffmpeg, "-y", "-framerate", str(FPS), "-i", f"{FRAMEDIR}/%05d.png",
           "-c:v", "libx264", "-pix_fmt", "yuv420p", "-crf", "20", OUT]
    print("+", " ".join(cmd))
    subprocess.run(cmd, check=True)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--all", action="store_true")
    a = ap.parse_args()
    if not a.all:
        print(__doc__)
        return 0
    render_all()
    assemble()
    return 0


if __name__ == "__main__":
    sys.exit(main())
