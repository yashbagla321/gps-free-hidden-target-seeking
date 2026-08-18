#!/usr/bin/env python3
"""Submission video pipeline (honestly labeled numerical simulation).

Steps:
  1. Build bin/viz_dump from src/viz_dump.cpp and dump the two
     representative traces (nominal seed 12; yaw step 60 deg at t=8 s).
  2. Render frames via scripts/video_render_chunk.py (chunked because a
     full render can exceed constrained-environment wall limits):
       python3 scripts/video_render_chunk.py 0 320   # ... repeat in 320-frame
       python3 scripts/video_render_chunk.py 320 640 # chunks through 1890
  3. Assemble:
       ffmpeg -y -framerate 30 -i <framedir>/%05d.png -c:v libx264 \
              -pix_fmt yuv420p -crf 23 \
              docs/campaign2027_submission/video/gps_free_seeking_simulation.mp4

Run from the gps_free_seeking project root. On an unconstrained machine:
  python3 scripts/make_video.py --all
"""

import argparse
import os
import subprocess
import sys

FRAMES = 1890  # 63 s at 30 fps
FRAMEDIR = "/tmp/vidframes"
OUT = "docs/campaign2027_submission/video/gps_free_seeking_simulation.mp4"


def run(cmd):
    print("+", " ".join(cmd))
    subprocess.run(cmd, check=True)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--all", action="store_true",
                    help="dump traces, render all frames, assemble")
    a = ap.parse_args()
    if not a.all:
        print(__doc__)
        return 0
    run(["g++", "-O2", "-std=c++17", "-Iinclude", "src/viz_dump.cpp",
         "-o", "bin/viz_dump"])
    run(["./bin/viz_dump", "/tmp/viz_nominal.csv", "nominal", "12"])
    run(["./bin/viz_dump", "/tmp/viz_yawstep.csv", "yawstep", "12"])
    os.makedirs(FRAMEDIR, exist_ok=True)
    run(["python3", "scripts/video_render_chunk.py", "0", str(FRAMES)])
    os.makedirs(os.path.dirname(OUT), exist_ok=True)
    run(["ffmpeg", "-y", "-framerate", "30", "-i",
         f"{FRAMEDIR}/%05d.png", "-c:v", "libx264", "-pix_fmt", "yuv420p",
         "-crf", "23", OUT])
    return 0


if __name__ == "__main__":
    sys.exit(main())
