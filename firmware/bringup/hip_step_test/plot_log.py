#!/usr/bin/env python3
"""Plot a hip_step_test log (picocom --logfile) -> PNG + clean CSV + per-step torque table.

usage: python3 plot_log.py ~/logs/run1.txt
"""
import sys
from pathlib import Path

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt  # noqa: E402

L_COLOR = "#2a78d6"   # categorical slot 1
R_COLOR = "#eb6834"   # categorical slot 2
INK = "#3a3a38"
GRID = "#d9d8d2"
COLS = ["t_ms", "L_ref", "L_pos", "L_vel", "L_tau", "R_ref", "R_pos", "R_vel", "R_tau"]


def load(path):
    rows = []
    with open(path, errors="ignore") as f:
        for line in f:
            line = line.strip()
            if not line.startswith("D,"):
                continue
            parts = line.split(",")[1:]
            if len(parts) == len(COLS):
                parts = parts + ["0"]                     # v1 log: no ff column
            if len(parts) != len(COLS) + 1:
                continue
            try:
                rows.append([float(x) for x in parts])
            except ValueError:
                continue
    return rows


def plateaus(rows, ref_idx, min_len=50):
    """Groups of samples where the reference is constant for >= min_len samples (0.5 s)."""
    out, start = [], 0
    for i in range(1, len(rows) + 1):
        if i == len(rows) or abs(rows[i][ref_idx] - rows[start][ref_idx]) > 1e-3:
            if i - start >= min_len:
                out.append((start, i))
            start = i
    return out


def table(rows, side):
    ri, pi, ti = COLS.index(f"{side}_ref"), COLS.index(f"{side}_pos"), COLS.index(f"{side}_tau")
    print(f"\n[{side}]  ref(deg)  pos(deg)  err(deg)  tau_mean(N*m)  tau_min  tau_max   ff")
    for a, b in plateaus(rows, ri):
        seg = rows[a + int(0.3 * (b - a)):b]          # skip the first 30 % (settling)
        ref = seg[0][ri]
        pos = sum(r[pi] for r in seg) / len(seg)
        taus = [r[ti] for r in seg]
        print(f"      {ref:+7.1f}  {pos:+8.2f}  {ref - pos:+8.2f}  {sum(taus) / len(taus):+13.3f}"
              f"  {min(taus):+7.3f}  {max(taus):+7.3f}  {seg[-1][-1]:+5.1f}")


def main():
    if len(sys.argv) < 2:
        sys.exit(__doc__)
    src = Path(sys.argv[1]).expanduser()
    rows = load(src)
    if not rows:
        sys.exit("no 'D,' lines found - did the sweep run while logging?")

    csv_path = src.with_suffix(".csv")
    with open(csv_path, "w") as f:
        f.write(",".join(COLS + ["ff"]) + "\n")
        for r in rows:
            f.write(",".join(f"{x:g}" for x in r) + "\n")

    t0 = rows[0][0]
    t = [(r[0] - t0) / 1000.0 for r in rows]
    col = {c: [r[i] for r in rows] for i, c in enumerate(COLS + ["ff"])}

    plt.rcParams.update({"font.size": 10, "axes.edgecolor": GRID, "axes.labelcolor": INK,
                         "xtick.color": INK, "ytick.color": INK, "text.color": INK})
    fig, ax = plt.subplots(3, 1, sharex=True, figsize=(11, 8.5), constrained_layout=True)

    for side, c in (("L", L_COLOR), ("R", R_COLOR)):
        ax[0].plot(t, col[f"{side}_ref"], color=c, lw=1.2, ls="--", label=f"{side} ref")
        ax[0].plot(t, col[f"{side}_pos"], color=c, lw=2, label=f"{side} pos")
        ax[1].plot(t, col[f"{side}_tau"], color=c, lw=2, label=f"{side}")
        err = [a - b for a, b in zip(col[f"{side}_ref"], col[f"{side}_pos"])]
        ax[2].plot(t, err, color=c, lw=2, label=f"{side}")

    if any(abs(v) > 1e-6 for v in col["ff"]):
        ax[1].plot(t, col["ff"], color="#8a8a85", lw=1.2, ls="--", label="ff (applied)")

    ax[0].set_ylabel("angle from base [deg]")
    ax[0].set_title(f"Hip step test - {src.name}", loc="left")
    ax[1].set_ylabel("torque [N*m]  (- = extend)")
    ax[2].set_ylabel("ref - pos [deg]")
    ax[2].set_xlabel("time [s]")
    for a in ax:
        a.grid(True, color=GRID, lw=0.6)
        a.axhline(0, color=GRID, lw=1)
        a.legend(loc="upper right", frameon=False, ncol=4)

    png = src.with_suffix(".png")
    fig.savefig(png, dpi=130)
    print(f"saved {png}  and  {csv_path}  ({len(rows)} samples, {t[-1]:.1f} s)")
    table(rows, "L")
    table(rows, "R")


if __name__ == "__main__":
    main()
