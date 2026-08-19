#!/usr/bin/env python3
"""Subscribe to the bridged Gazebo camera topic and write an MP4 clip.

Illustrative-footage tool only: it does not touch the estimator, controller,
or any citable statistic. Exits automatically in one of three ways:

  * ``--duration`` sim-time seconds of frames have been captured;
  * ``--timeout`` wall-clock seconds pass with no first frame at all
    (wrong world/bridge, camera sensor never came up);
  * ``--source-quiet-timeout`` wall-clock seconds pass with no new frame
    after recording has already started. This covers the normal shutdown
    path: when the mission's launch (and therefore the Gazebo process and
    ROS-GZ bridge) exits before ``--duration`` is reached, the ``/camera``
    topic simply stops publishing with no ROS-level "source gone" signal,
    so without this watchdog the node spins forever waiting for frames
    that will never arrive.

Pass ``--frames-csv`` to also log each written frame's index and sim-time
timestamp, so a downstream compositor (see ../../../../scripts/
make_gazebo_video.py) can line frames up against ground-truth telemetry.

Usage:
    python3 capture_camera_video.py --out clip.mp4 --duration 15 --fps 30 \
        --frames-csv clip.frames.csv
"""

import argparse
import sys
import time

import cv2
import rclpy
from cv_bridge import CvBridge
from rclpy.node import Node
from sensor_msgs.msg import Image


class CameraCapture(Node):
    def __init__(self, out_path, duration, fps, frames_csv=None):
        super().__init__("camera_capture")
        self.bridge = CvBridge()
        self.writer = None
        self.out_path = out_path
        self.duration = duration
        self.fps = fps
        self.first_stamp = None
        self.frame_count = 0
        self.frames_csv = frames_csv
        self.frames_log = []
        self.last_frame_wall = None
        self.sub = self.create_subscription(
            Image, "/camera", self.on_image, 10)

    def on_image(self, msg):
        t = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        self.last_frame_wall = time.time()
        if self.first_stamp is None:
            self.first_stamp = t
            self.get_logger().info(f"first frame at sim t={t:.2f}s")
        frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        if self.writer is None:
            h, w = frame.shape[:2]
            fourcc = cv2.VideoWriter_fourcc(*"mp4v")
            self.writer = cv2.VideoWriter(self.out_path, fourcc, self.fps,
                                           (w, h))
            self.get_logger().info(f"recording {w}x{h} -> {self.out_path}")
        self.writer.write(frame)
        self.frames_log.append((self.frame_count, t))
        self.frame_count += 1
        if t - self.first_stamp >= self.duration:
            self.get_logger().info(
                f"captured {self.frame_count} frames "
                f"({t - self.first_stamp:.1f}s sim time), stopping")
            rclpy.shutdown()

    def finish(self):
        if self.writer is not None:
            self.writer.release()
        if self.frames_csv is not None:
            with open(self.frames_csv, "w") as f:
                f.write("frame_idx,sim_t\n")
                for idx, t in self.frames_log:
                    f.write(f"{idx},{t}\n")


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--out", required=True)
    ap.add_argument("--duration", type=float, default=15.0,
                     help="sim-time seconds of footage to capture")
    ap.add_argument("--fps", type=float, default=30.0)
    ap.add_argument("--timeout", type=float, default=60.0,
                     help="wall-clock seconds to wait for the first frame")
    ap.add_argument("--frames-csv", default=None,
                     help="optional path to log frame_idx,sim_t per frame")
    ap.add_argument("--source-quiet-timeout", type=float, default=8.0,
                     help="wall-clock seconds with no new frame (after "
                          "recording has started) before treating the "
                          "source as finished and stopping cleanly")
    args = ap.parse_args()

    rclpy.init()
    node = CameraCapture(args.out, args.duration, args.fps, args.frames_csv)
    start = time.time()
    try:
        while rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.5)
            if node.first_stamp is None and time.time() - start > args.timeout:
                node.get_logger().error(
                    f"no frames received within {args.timeout}s, giving up")
                sys.exit(1)
            if (node.last_frame_wall is not None and
                    time.time() - node.last_frame_wall >
                    args.source_quiet_timeout):
                node.get_logger().info(
                    f"no new frame for {args.source_quiet_timeout}s "
                    f"(source likely shut down), stopping with "
                    f"{node.frame_count} frames captured")
                break
    except KeyboardInterrupt:
        pass
    finally:
        node.finish()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
