#!/usr/bin/env python3
"""
AprilTag angle reader for AS5147P encoder calibration.

Detects AprilTag(s) in the webcam feed and reports the in-plane rotation
angle in degrees. Use as a ground-truth reference to calibrate a magnetic
rotary encoder mounted on the same shaft.

Setup:
  1. Print an AprilTag (tag36h11 or tag16h5, ID 0 by default).
     Pre-rendered PNGs: https://github.com/AprilRobotics/apriltag-imgs
  2. Mount the tag concentric with the rotor face. Off-center mounting
     gives you a once-per-rev sinusoidal error that will look like the
     encoder being non-linear.
  3. Aim the webcam straight down the rotation axis so the tag stays
     square in frame as it spins.
  4. Run: python anglereader.py

Install:
  pip install opencv-contrib-python numpy matplotlib

Keys (with the OpenCV window focused):
  q  quit
  z  zero the angle (current cumulative becomes 0 deg, per tag)
  r  reset cumulative tracking (clears unwrap state)
  s  save current frame as PNG
  l  toggle CSV logging (writes apriltag_log.csv in cwd)
  p  toggle live plot window

Sign convention: angle increases counter-clockwise as seen by the camera.
Use --invert if your encoder counts the other way.
"""

import argparse
import csv
import math
import sys
import time
from collections import deque

import cv2
import numpy as np


# --- detection -----------------------------------------------------------

APRILTAG_FAMILIES = {
    "36h11": cv2.aruco.DICT_APRILTAG_36h11,
    "25h9":  cv2.aruco.DICT_APRILTAG_25h9,
    "16h5":  cv2.aruco.DICT_APRILTAG_16h5,
}


def make_detector(family: str):
    if family not in APRILTAG_FAMILIES:
        raise ValueError(f"unknown family {family!r}; choose from "
                         f"{list(APRILTAG_FAMILIES)}")
    aruco_dict = cv2.aruco.getPredefinedDictionary(APRILTAG_FAMILIES[family])
    params = cv2.aruco.DetectorParameters()
    params.cornerRefinementMethod = cv2.aruco.CORNER_REFINE_SUBPIX
    return cv2.aruco.ArucoDetector(aruco_dict, params)


def tag_angle_deg(corners4: np.ndarray, invert: bool) -> float:
    """In-plane rotation, degrees, range (-180, 180].

    corners4 shape (4, 2): [TL, TR, BR, BL] when unrotated. We atan2 the
    top-edge vector (corner 0 -> corner 1). Image y points down, so we
    negate dy to make CCW positive in the human sense.
    """
    p0 = corners4[0]
    p1 = corners4[1]
    dx = float(p1[0] - p0[0])
    dy = float(p1[1] - p0[1])
    angle = math.degrees(math.atan2(-dy, dx))
    if invert:
        angle = -angle
    return angle


# --- cumulative angle tracking ------------------------------------------

class AngleTracker:
    """Unwraps a wrapped (-180, 180] angle stream into a continuous one."""

    def __init__(self):
        self.prev_raw = None
        self.cumulative = 0.0

    def reset(self):
        self.prev_raw = None
        self.cumulative = 0.0

    def update(self, raw_deg: float) -> float:
        if self.prev_raw is None:
            self.prev_raw = raw_deg
            return self.cumulative
        delta = raw_deg - self.prev_raw
        if delta > 180.0:
            delta -= 360.0
        elif delta < -180.0:
            delta += 360.0
        self.cumulative += delta
        self.prev_raw = raw_deg
        return self.cumulative


# --- live plot ----------------------------------------------------------

class LivePlot:
    """Lightweight matplotlib live plot. One line per tag id, angle vs time.

    Matplotlib is imported lazily so the main script still runs if it's
    not installed and you don't ask for plotting.
    """

    def __init__(self, window_seconds: float = 30.0, max_points: int = 3000):
        import matplotlib
        matplotlib.use("TkAgg")  # interactive backend; works alongside cv2
        import matplotlib.pyplot as plt
        self._plt = plt

        self.window_seconds = window_seconds
        self.max_points = max_points
        self.t0 = None

        # per-tag deques of (t, angle_zeroed)
        self.data = {}  # tag_id -> (deque[t], deque[angle])
        self.lines = {}  # tag_id -> Line2D

        plt.ion()
        self.fig, self.ax = plt.subplots(figsize=(8, 4))
        self.ax.set_xlabel("time (s)")
        self.ax.set_ylabel("zeroed angle (deg)")
        self.ax.set_title("AprilTag angle (live)")
        self.ax.grid(True, alpha=0.3)
        self.fig.tight_layout()
        self.fig.canvas.mpl_connect("close_event", self._on_close)
        self._closed = False
        self._last_draw = 0.0

    def _on_close(self, _event):
        self._closed = True

    @property
    def closed(self) -> bool:
        return self._closed

    def push(self, tag_id: int, t_abs: float, angle_zeroed: float):
        if self._closed:
            return
        if self.t0 is None:
            self.t0 = t_abs
        t = t_abs - self.t0
        if tag_id not in self.data:
            ts = deque(maxlen=self.max_points)
            ys = deque(maxlen=self.max_points)
            self.data[tag_id] = (ts, ys)
            (line,) = self.ax.plot([], [], label=f"id {tag_id}")
            self.lines[tag_id] = line
            self.ax.legend(loc="upper left")
        ts, ys = self.data[tag_id]
        ts.append(t)
        ys.append(angle_zeroed)

    def redraw(self, force: bool = False):
        """Cheap redraw, throttled to ~20 Hz to avoid stalling capture."""
        if self._closed:
            return
        now = time.time()
        if not force and (now - self._last_draw) < 0.05:
            return
        self._last_draw = now

        any_data = False
        x_max = 0.0
        for tag_id, (ts, ys) in self.data.items():
            if not ts:
                continue
            any_data = True
            self.lines[tag_id].set_data(ts, ys)
            x_max = max(x_max, ts[-1])

        if any_data:
            x_lo = max(0.0, x_max - self.window_seconds)
            self.ax.set_xlim(x_lo, max(x_lo + 1e-3, x_max))
            self.ax.relim()
            self.ax.autoscale_view(scalex=False, scaley=True)

        try:
            self.fig.canvas.draw_idle()
            self.fig.canvas.flush_events()
        except Exception:
            # window was closed mid-draw
            self._closed = True

    def close(self):
        if self._closed:
            return
        self._closed = True
        try:
            self._plt.close(self.fig)
        except Exception:
            pass


# --- main ---------------------------------------------------------------

def main():
    ap = argparse.ArgumentParser(
        description=__doc__,
        formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--camera", type=int, default=1, help="webcam index (default 1)")
    ap.add_argument("--family", default="16h5",
                    choices=list(APRILTAG_FAMILIES),
                    help="AprilTag family (default 16h5)")
    ap.add_argument("--width", type=int, default=1280, help="capture width")
    ap.add_argument("--height", type=int, default=720, help="capture height")
    ap.add_argument("--invert", action="store_true",
                    help="invert sign of angle (use if encoder counts opposite)")
    ap.add_argument("--csv", default="apriltag_log.csv",
                    help="CSV path for logging (toggle with 'l' key)")
    ap.add_argument("--log-on-start", action="store_true",
                    help="start CSV logging immediately on launch")
    ap.add_argument("--plot", action="store_true",
                    help="open live plot window on launch")
    ap.add_argument("--plot-window", type=float, default=30.0,
                    help="live plot rolling window in seconds (default 30)")
    ap.add_argument("--no-display", action="store_true",
                    help="run headless, print only (disables plot too)")
    args = ap.parse_args()

    cap = cv2.VideoCapture(args.camera)
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, args.width)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, args.height)
    if not cap.isOpened():
        print(f"ERROR: cannot open camera index {args.camera}", file=sys.stderr)
        sys.exit(1)

    detector = make_detector(args.family)
    trackers: dict[int, AngleTracker] = {}
    zero_offsets: dict[int, float] = {}

    # --- CSV logging ---
    log_file = None
    log_writer = None

    def start_log():
        nonlocal log_file, log_writer
        log_file = open(args.csv, "a", newline="")
        log_writer = csv.writer(log_file)
        log_file.seek(0, 2)
        if log_file.tell() == 0:
            log_writer.writerow(["time_s", "tag_id",
                                 "angle_raw_deg", "angle_zeroed_deg"])
        print(f"\n[log started -> {args.csv}]")

    def stop_log():
        nonlocal log_file, log_writer
        if log_file:
            log_file.close()
        log_file = None
        log_writer = None
        print("\n[log stopped]")

    if args.log_on_start:
        start_log()

    # --- live plot ---
    live = None
    if args.plot and not args.no_display:
        try:
            live = LivePlot(window_seconds=args.plot_window)
            print("[plot enabled]")
        except Exception as e:
            print(f"[plot disabled: {e}]", file=sys.stderr)
            live = None

    print("AprilTag angle reader. Keys: q=quit z=zero r=reset s=save l=log p=plot")
    last_term_print = 0.0

    try:
        while True:
            ok, frame = cap.read()
            if not ok:
                print("frame grab failed", file=sys.stderr)
                break

            gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
            corners_list, ids, _ = detector.detectMarkers(gray)

            now = time.time()
            results = []  # list of (tag_id, angle_raw, angle_zeroed)

            if ids is not None and len(ids) > 0:
                ids_flat = ids.flatten()
                for i, tag_id in enumerate(ids_flat):
                    tag_id = int(tag_id)
                    c4 = corners_list[i][0]
                    angle_raw = tag_angle_deg(c4, args.invert)
                    if tag_id not in trackers:
                        trackers[tag_id] = AngleTracker()
                    cumulative = trackers[tag_id].update(angle_raw)
                    angle_zeroed = cumulative - zero_offsets.get(tag_id, 0.0)
                    results.append((tag_id, angle_raw, angle_zeroed))
                    if not args.no_display:
                        cv2.aruco.drawDetectedMarkers(
                            frame, [corners_list[i]],
                            np.array([[tag_id]]))

            # --- overlay ---
            if not args.no_display:
                if results:
                    for row_i, (tag_id, angle_raw, angle_zeroed) in enumerate(results):
                        txt = (f"id={tag_id}  raw={angle_raw:+7.2f}  "
                               f"zeroed={angle_zeroed:+9.2f} deg")
                        cv2.putText(frame, txt, (10, 30 + row_i * 30),
                                    cv2.FONT_HERSHEY_SIMPLEX, 0.7,
                                    (0, 255, 0), 2, cv2.LINE_AA)
                else:
                    cv2.putText(frame, "no tag", (10, 30),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.8,
                                (0, 0, 255), 2, cv2.LINE_AA)

                badges = []
                if log_writer is not None:
                    badges.append(("LOG", (0, 165, 255)))
                if live is not None and not live.closed:
                    badges.append(("PLOT", (255, 200, 0)))
                for bi, (txt, color) in enumerate(badges):
                    cv2.putText(frame, txt,
                                (10, 30 + len(results) * 30 + 30 + bi * 25),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.6,
                                color, 2, cv2.LINE_AA)

                cv2.imshow("apriltag angle", frame)

            # --- terminal print (throttled to 10 Hz) ---
            if now - last_term_print > 0.1:
                if results:
                    line = "  |  ".join(
                        f"id={t} raw={r:+7.2f} zeroed={z:+9.2f}"
                        for t, r, z in results)
                    sys.stdout.write(f"\r{line}   ")
                else:
                    sys.stdout.write("\rno tag                                  ")
                sys.stdout.flush()
                last_term_print = now

            # --- CSV ---
            if log_writer is not None:
                for tag_id, angle_raw, angle_zeroed in results:
                    log_writer.writerow([f"{now:.6f}", tag_id,
                                         f"{angle_raw:.4f}",
                                         f"{angle_zeroed:.4f}"])

            # --- plot push & redraw ---
            if live is not None and not live.closed:
                for tag_id, _angle_raw, angle_zeroed in results:
                    live.push(tag_id, now, angle_zeroed)
                live.redraw()
            elif live is not None and live.closed:
                # user closed the plot window; drop the handle so 'p' can reopen
                live = None

            # --- keys ---
            key = cv2.waitKey(1) & 0xFF if not args.no_display else 0xFF
            if key == ord('q'):
                break
            elif key == ord('z'):
                for tag_id, _, angle_zeroed in results:
                    zero_offsets[tag_id] = zero_offsets.get(tag_id, 0.0) + angle_zeroed
                print("\n[zeroed all]")
            elif key == ord('r'):
                trackers.clear()
                zero_offsets.clear()
                print("\n[tracker reset]")
            elif key == ord('s'):
                fn = f"frame_{int(now)}.png"
                cv2.imwrite(fn, frame)
                print(f"\n[saved {fn}]")
            elif key == ord('l'):
                if log_writer is None:
                    start_log()
                else:
                    stop_log()
            elif key == ord('p'):
                if live is None or live.closed:
                    try:
                        live = LivePlot(window_seconds=args.plot_window)
                        print("\n[plot opened]")
                    except Exception as e:
                        print(f"\n[plot failed: {e}]", file=sys.stderr)
                        live = None
                else:
                    live.close()
                    live = None
                    print("\n[plot closed]")

    finally:
        if log_file:
            log_file.close()
        if live is not None:
            live.close()
        cap.release()
        if not args.no_display:
            cv2.destroyAllWindows()
        print()


if __name__ == "__main__":
    main()
