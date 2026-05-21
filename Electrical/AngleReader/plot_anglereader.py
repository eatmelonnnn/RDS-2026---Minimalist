#!/usr/bin/env python3
"""
Plot the CSV produced by anglereader.py.

Default: angle vs time for each tag id, plus a histogram of frame-to-frame
angle deltas so you can eyeball detection glitches.

If you also pass --encoder-csv (a CSV with columns time_s,angle_deg from
your AS5147P logger), it will resample both streams onto a common time
base and plot apriltag vs encoder, plus the residual.

Examples:
  python plot_anglereader.py apriltag_log.csv
  python plot_anglereader.py apriltag_log.csv --tag 0
  python plot_anglereader.py apriltag_log.csv --encoder-csv encoder_log.csv --tag 0
  python plot_anglereader.py apriltag_log.csv --save out.png
"""

import argparse
import csv
import sys
from collections import defaultdict

import numpy as np
import matplotlib.pyplot as plt


def load_apriltag_csv(path):
    """Returns dict: tag_id -> (t[np.ndarray], raw[np.ndarray], zeroed[np.ndarray])."""
    by_tag = defaultdict(lambda: ([], [], []))
    with open(path, newline="") as f:
        reader = csv.DictReader(f)
        required = {"time_s", "tag_id", "angle_raw_deg", "angle_zeroed_deg"}
        if not required.issubset(reader.fieldnames or []):
            raise ValueError(
                f"{path}: expected columns {sorted(required)}, "
                f"got {reader.fieldnames}")
        for row in reader:
            tid = int(row["tag_id"])
            by_tag[tid][0].append(float(row["time_s"]))
            by_tag[tid][1].append(float(row["angle_raw_deg"]))
            by_tag[tid][2].append(float(row["angle_zeroed_deg"]))
    out = {}
    for tid, (t, r, z) in by_tag.items():
        out[tid] = (np.asarray(t), np.asarray(r), np.asarray(z))
    return out


def load_encoder_csv(path):
    """Encoder CSV: time_s,angle_deg. Returns (t, angle)."""
    ts, ys = [], []
    with open(path, newline="") as f:
        reader = csv.DictReader(f)
        cols = reader.fieldnames or []
        if "time_s" not in cols or "angle_deg" not in cols:
            raise ValueError(
                f"{path}: expected columns time_s,angle_deg; got {cols}")
        for row in reader:
            ts.append(float(row["time_s"]))
            ys.append(float(row["angle_deg"]))
    return np.asarray(ts), np.asarray(ys)


def plot_single_source(by_tag, only_tag=None, save_path=None):
    if only_tag is not None:
        by_tag = {only_tag: by_tag[only_tag]}

    fig, (ax_t, ax_h) = plt.subplots(
        2, 1, figsize=(10, 7),
        gridspec_kw={"height_ratios": [3, 1]})

    t_min = min(t[0] for t, _, _ in by_tag.values())

    for tid, (t, _raw, z) in sorted(by_tag.items()):
        ax_t.plot(t - t_min, z, label=f"id {tid}", linewidth=1)

    ax_t.set_xlabel("time (s)")
    ax_t.set_ylabel("zeroed angle (deg)")
    ax_t.set_title("AprilTag angle vs time")
    ax_t.grid(True, alpha=0.3)
    ax_t.legend(loc="best")

    # frame-to-frame deltas, helps spot detection dropouts / jumps
    for tid, (t, _raw, z) in sorted(by_tag.items()):
        if len(z) < 2:
            continue
        dz = np.diff(z)
        ax_h.hist(dz, bins=80, alpha=0.5, label=f"id {tid}")
    ax_h.set_xlabel("frame-to-frame Δangle (deg)")
    ax_h.set_ylabel("count")
    ax_h.set_title("Δangle histogram (look for outliers = detection glitches)")
    ax_h.grid(True, alpha=0.3)
    ax_h.legend(loc="best")

    # quick stats
    print("\nPer-tag stats:")
    print(f"  {'id':>4}  {'samples':>8}  {'dur(s)':>8}  "
          f"{'mean dt(ms)':>11}  {'range(deg)':>11}")
    for tid, (t, _raw, z) in sorted(by_tag.items()):
        if len(t) < 2:
            continue
        dt_ms = np.mean(np.diff(t)) * 1000.0
        dur = t[-1] - t[0]
        rng = z.max() - z.min()
        print(f"  {tid:>4}  {len(t):>8}  {dur:>8.2f}  "
              f"{dt_ms:>11.2f}  {rng:>11.2f}")

    fig.tight_layout()
    if save_path:
        fig.savefig(save_path, dpi=120)
        print(f"\nsaved -> {save_path}")
    return fig


def plot_with_encoder(by_tag, enc_t, enc_y, only_tag, save_path=None):
    if only_tag is None:
        # pick the tag with the most samples
        only_tag = max(by_tag.items(), key=lambda kv: len(kv[1][0]))[0]
        print(f"--tag not given; using id {only_tag} (most samples)")

    t, _raw, z = by_tag[only_tag]

    # common time base = overlap of the two streams, sampled at the
    # apriltag rate (typically the slower of the two)
    t0 = max(t[0], enc_t[0])
    t1 = min(t[-1], enc_t[-1])
    if t1 <= t0:
        raise ValueError("encoder and apriltag logs do not overlap in time")

    mask = (t >= t0) & (t <= t1)
    t_common = t[mask]
    apt_common = z[mask]
    enc_common = np.interp(t_common, enc_t, enc_y)

    # remove constant offset between them (zero at first overlapping sample)
    apt_common = apt_common - apt_common[0]
    enc_common = enc_common - enc_common[0]
    residual = enc_common - apt_common

    fig, (ax_both, ax_res) = plt.subplots(
        2, 1, figsize=(10, 7), sharex=True,
        gridspec_kw={"height_ratios": [2, 1]})

    tt = t_common - t_common[0]
    ax_both.plot(tt, apt_common, label=f"AprilTag id {only_tag}", linewidth=1)
    ax_both.plot(tt, enc_common, label="Encoder", linewidth=1, alpha=0.8)
    ax_both.set_ylabel("angle (deg)")
    ax_both.set_title("AprilTag vs encoder (zeroed at overlap start)")
    ax_both.legend(loc="best")
    ax_both.grid(True, alpha=0.3)

    ax_res.plot(tt, residual, color="crimson", linewidth=1)
    ax_res.axhline(0.0, color="k", linewidth=0.5)
    ax_res.set_xlabel("time (s)")
    ax_res.set_ylabel("encoder − apriltag (deg)")
    ax_res.set_title(
        f"residual  |  mean={residual.mean():+.3f}  "
        f"std={residual.std():.3f}  pk-pk={np.ptp(residual):.3f}")
    ax_res.grid(True, alpha=0.3)

    print(f"\nResidual stats (encoder - apriltag, n={len(residual)}):")
    print(f"  mean   {residual.mean():+.4f} deg")
    print(f"  std    {residual.std():.4f} deg")
    print(f"  min    {residual.min():+.4f} deg")
    print(f"  max    {residual.max():+.4f} deg")
    print(f"  pk-pk  {np.ptp(residual):.4f} deg")

    fig.tight_layout()
    if save_path:
        fig.savefig(save_path, dpi=120)
        print(f"\nsaved -> {save_path}")
    return fig


def main():
    ap = argparse.ArgumentParser(
        description=__doc__,
        formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("csv", help="apriltag CSV from anglereader.py")
    ap.add_argument("--tag", type=int, default=None,
                    help="only plot this tag id (default: all)")
    ap.add_argument("--encoder-csv", default=None,
                    help="optional encoder CSV with columns time_s,angle_deg")
    ap.add_argument("--save", default=None,
                    help="save figure to this path instead of (or in addition to) showing")
    ap.add_argument("--no-show", action="store_true",
                    help="don't pop up the interactive window")
    args = ap.parse_args()

    by_tag = load_apriltag_csv(args.csv)
    if not by_tag:
        print(f"{args.csv}: no rows", file=sys.stderr)
        sys.exit(1)

    if args.encoder_csv:
        enc_t, enc_y = load_encoder_csv(args.encoder_csv)
        plot_with_encoder(by_tag, enc_t, enc_y, args.tag, save_path=args.save)
    else:
        plot_single_source(by_tag, only_tag=args.tag, save_path=args.save)

    if not args.no_show:
        plt.show()


if __name__ == "__main__":
    main()
