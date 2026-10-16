#!/usr/bin/env python3
"""Plot motion-risk comparisons between two captured sessions.

This script compares:
- an older baseline session (for example 1024x10)
- a newer session (for example 4096x5)
- a hypothetical "newer session if captured at a different lidar mode"

The goal is to make the scan-duration effect visible using recorded GPS motion:
- linear distance traveled during one full LiDAR revolution
- yaw rotation during one full LiDAR revolution

Those two quantities are the most direct "why 4096x5 is touchier" plots for
mobile mapping without needing a full SLAM rerun.
"""

from __future__ import annotations

import argparse
import importlib.util
import json
import math
import re
from dataclasses import dataclass
from pathlib import Path
from typing import Any

import matplotlib.pyplot as plt
import numpy as np
import pandas as pd


REPO_ROOT = Path(__file__).resolve().parent
SAMPLE_DATA_ROOT = Path("/home/spriteadmin/Documents/LiDAR-Object-Detection/SampleData")
DEFAULT_OLD_SESSION_ID = "20260317_101058"
DEFAULT_NEW_SESSION_ID = "20260616_173132"
DEFAULT_HYPOTHETICAL_MODE = "1024x10"


@dataclass
class SessionMotion:
    session_id: str
    session_dir: Path
    label: str
    lidar_mode: str
    revolution_period_s: float
    raw_gps_path: Path
    metadata_path: Path | None
    gps_df: pd.DataFrame
    time_col: str
    segment_time_min: np.ndarray
    speed_mps: np.ndarray
    yaw_time_min: np.ndarray
    yaw_rate_deg_s: np.ndarray
    distance_per_rev_m: np.ndarray
    turn_per_rev_deg: np.ndarray


def _import_offline_fusion() -> Any:
    module_path = REPO_ROOT / "src/lidar_rpi_gps_pipeline/jobs/offline_fusion.py"
    spec = importlib.util.spec_from_file_location("offline_fusion", module_path)
    if spec is None or spec.loader is None:
        raise RuntimeError(f"Could not import offline_fusion from {module_path}")
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def _find_session_dir(session_id: str) -> Path:
    matches = sorted(
        path
        for path in SAMPLE_DATA_ROOT.iterdir()
        if path.is_dir() and (path / f"raw_gps_{session_id}.csv").exists()
    )
    if not matches:
        raise FileNotFoundError(
            f"Could not find a SampleData folder containing raw_gps_{session_id}.csv"
        )
    return matches[0]


def _find_session_metadata(session_dir: Path, session_id: str) -> Path | None:
    matches = sorted(session_dir.glob(f"raw_lidar_{session_id}_chunk*_0.json"))
    return matches[0] if matches else None


def _load_lidar_mode(metadata_path: Path | None) -> str | None:
    if metadata_path is None:
        return None
    data = json.loads(metadata_path.read_text())
    return data.get("lidar_mode") or data.get("config_params", {}).get("lidar_mode")


def _parse_mode(mode: str) -> tuple[int, int]:
    match = re.fullmatch(r"(\d+)x(\d+)", mode.strip())
    if not match:
        raise ValueError(f"Unsupported lidar mode format: {mode!r}")
    return int(match.group(1)), int(match.group(2))


def _revolution_period_s(mode: str) -> float:
    _, hz = _parse_mode(mode)
    if hz <= 0:
        raise ValueError(f"Invalid lidar mode frequency in {mode!r}")
    return 1.0 / float(hz)


def _finite_nonempty(values: np.ndarray) -> np.ndarray:
    arr = np.asarray(values, dtype=float)
    return arr[np.isfinite(arr)]


def _safe_percentile(values: np.ndarray, percentile: float) -> float | None:
    arr = _finite_nonempty(values)
    if arr.size == 0:
        return None
    return float(np.percentile(arr, percentile))


def _safe_median(values: np.ndarray) -> float | None:
    arr = _finite_nonempty(values)
    if arr.size == 0:
        return None
    return float(np.median(arr))


def _compute_heading_metrics(
    easting: np.ndarray,
    northing: np.ndarray,
    time_ns: np.ndarray,
    min_step_m: float = 0.5,
) -> tuple[np.ndarray, np.ndarray]:
    dx = np.diff(easting)
    dy = np.diff(northing)
    dt = np.diff(time_ns).astype(np.float64) / 1e9
    segment_time_ns = (time_ns[:-1] + time_ns[1:]) / 2.0
    distance_m = np.hypot(dx, dy)

    valid = (dt > 0.0) & np.isfinite(dt) & np.isfinite(distance_m) & (distance_m >= min_step_m)
    if np.count_nonzero(valid) < 3:
        return np.asarray([], dtype=np.float64), np.asarray([], dtype=np.float64)

    heading_rad = np.unwrap(np.arctan2(dy[valid], dx[valid]))
    heading_time_ns = segment_time_ns[valid]

    delta_heading_rad = np.diff(heading_rad)
    delta_time_s = np.diff(heading_time_ns).astype(np.float64) / 1e9
    valid_rate = delta_time_s > 0.0
    if np.count_nonzero(valid_rate) == 0:
        return np.asarray([], dtype=np.float64), np.asarray([], dtype=np.float64)

    yaw_rate_deg_s = np.abs(np.rad2deg(delta_heading_rad[valid_rate] / delta_time_s[valid_rate]))
    yaw_time_ns = (heading_time_ns[:-1][valid_rate] + heading_time_ns[1:][valid_rate]) / 2.0
    yaw_time_min = (yaw_time_ns - yaw_time_ns[0]) / 60e9
    return yaw_time_min.astype(np.float64), yaw_rate_deg_s.astype(np.float64)


def _load_session_motion(
    session_id: str,
    label: str,
    fallback_mode: str | None,
    max_speed_mps: float,
) -> SessionMotion:
    offline_fusion = _import_offline_fusion()
    session_dir = _find_session_dir(session_id)
    raw_gps_path = session_dir / f"raw_gps_{session_id}.csv"
    metadata_path = _find_session_metadata(session_dir, session_id)
    lidar_mode = _load_lidar_mode(metadata_path) or fallback_mode
    if not lidar_mode:
        raise RuntimeError(
            f"Could not determine lidar mode for session {session_id}. "
            "Provide --old-mode/--new-mode explicitly."
        )

    gps_df, time_col = offline_fusion.load_gps_for_interpolation(
        str(raw_gps_path),
        "gps_epoch_ns",
        preferred_auto_columns=["gps_epoch_ns"],
        apply_outlier_filter=True,
        gps_outlier_max_speed_mps=max_speed_mps,
    )
    if len(gps_df) < 3:
        raise RuntimeError(f"Session {session_id} does not have enough GPS rows after filtering.")

    easting = gps_df["easting"].to_numpy(dtype=np.float64)
    northing = gps_df["northing"].to_numpy(dtype=np.float64)
    time_ns = gps_df[time_col].to_numpy(dtype=np.int64)

    dx = np.diff(easting)
    dy = np.diff(northing)
    dt_s = np.diff(time_ns).astype(np.float64) / 1e9
    valid_speed = dt_s > 0.0
    segment_time_ns = (time_ns[:-1] + time_ns[1:]) / 2.0
    segment_time_min = (segment_time_ns - segment_time_ns[0]) / 60e9
    speed_mps = np.hypot(dx, dy)[valid_speed] / dt_s[valid_speed]
    segment_time_min = segment_time_min[valid_speed]

    yaw_time_min, yaw_rate_deg_s = _compute_heading_metrics(easting, northing, time_ns)
    revolution_period_s = _revolution_period_s(lidar_mode)

    return SessionMotion(
        session_id=session_id,
        session_dir=session_dir,
        label=label,
        lidar_mode=lidar_mode,
        revolution_period_s=revolution_period_s,
        raw_gps_path=raw_gps_path,
        metadata_path=metadata_path,
        gps_df=gps_df,
        time_col=time_col,
        segment_time_min=segment_time_min,
        speed_mps=speed_mps,
        yaw_time_min=yaw_time_min,
        yaw_rate_deg_s=yaw_rate_deg_s,
        distance_per_rev_m=speed_mps * revolution_period_s,
        turn_per_rev_deg=yaw_rate_deg_s * revolution_period_s,
    )


def _make_variant(session: SessionMotion, label: str, lidar_mode: str) -> SessionMotion:
    revolution_period_s = _revolution_period_s(lidar_mode)
    return SessionMotion(
        session_id=session.session_id,
        session_dir=session.session_dir,
        label=label,
        lidar_mode=lidar_mode,
        revolution_period_s=revolution_period_s,
        raw_gps_path=session.raw_gps_path,
        metadata_path=session.metadata_path,
        gps_df=session.gps_df,
        time_col=session.time_col,
        segment_time_min=session.segment_time_min,
        speed_mps=session.speed_mps,
        yaw_time_min=session.yaw_time_min,
        yaw_rate_deg_s=session.yaw_rate_deg_s,
        distance_per_rev_m=session.speed_mps * revolution_period_s,
        turn_per_rev_deg=session.yaw_rate_deg_s * revolution_period_s,
    )


def _summary_rows(*sessions: SessionMotion) -> pd.DataFrame:
    rows = []
    for session in sessions:
        rows.append(
            {
                "label": session.label,
                "session_id": session.session_id,
                "session_dir": str(session.session_dir),
                "gps_csv_path": str(session.raw_gps_path),
                "metadata_path": str(session.metadata_path) if session.metadata_path else "",
                "lidar_mode": session.lidar_mode,
                "scan_period_s": session.revolution_period_s,
                "gps_rows_after_filter": int(len(session.gps_df)),
                "speed_median_mps": _safe_median(session.speed_mps),
                "speed_p95_mps": _safe_percentile(session.speed_mps, 95),
                "turn_rate_median_deg_s": _safe_median(session.yaw_rate_deg_s),
                "turn_rate_p95_deg_s": _safe_percentile(session.yaw_rate_deg_s, 95),
                "distance_per_rev_median_m": _safe_median(session.distance_per_rev_m),
                "distance_per_rev_p95_m": _safe_percentile(session.distance_per_rev_m, 95),
                "turn_per_rev_median_deg": _safe_median(session.turn_per_rev_deg),
                "turn_per_rev_p95_deg": _safe_percentile(session.turn_per_rev_deg, 95),
            }
        )
    return pd.DataFrame(rows)


def _plot_speed(ax: Any, sessions: list[SessionMotion]) -> None:
    for session, color in zip(sessions, ["#1f77b4", "#ff7f0e"], strict=True):
        ax.plot(
            session.segment_time_min,
            session.speed_mps,
            color=color,
            alpha=0.7,
            linewidth=1.3,
            label=f"{session.label} ({session.lidar_mode})",
        )
    ax.set_title("GPS Speed Over Time")
    ax.set_xlabel("Minutes From Session Start")
    ax.set_ylabel("Speed (m/s)")
    ax.grid(True, alpha=0.25)
    ax.legend(frameon=False, fontsize=9)


def _plot_turn_rate(ax: Any, sessions: list[SessionMotion]) -> None:
    ymax = 0.0
    for session, color in zip(sessions, ["#1f77b4", "#ff7f0e"], strict=True):
        arr = _finite_nonempty(session.yaw_rate_deg_s)
        if arr.size == 0:
            continue
        ymax = max(ymax, float(np.percentile(arr, 99)))
        ax.plot(
            session.yaw_time_min,
            session.yaw_rate_deg_s,
            color=color,
            alpha=0.7,
            linewidth=1.2,
            label=f"{session.label} ({session.lidar_mode})",
        )
    if ymax > 0:
        ax.set_ylim(0, ymax * 1.1)
    ax.set_title("GPS-Derived Turn Rate Proxy")
    ax.set_xlabel("Minutes From Session Start")
    ax.set_ylabel("Abs Yaw Rate (deg/s)")
    ax.grid(True, alpha=0.25)
    ax.legend(frameon=False, fontsize=9)


def _boxplot(ax: Any, sessions: list[SessionMotion], values_key: str, title: str, ylabel: str) -> None:
    colors = ["#1f77b4", "#ff7f0e", "#2ca02c"]
    series = []
    labels = []
    for session in sessions:
        values = _finite_nonempty(getattr(session, values_key))
        if values.size == 0:
            continue
        series.append(values)
        labels.append(f"{session.label}\n{session.lidar_mode}")
    if not series:
        ax.text(0.5, 0.5, "No valid data", ha="center", va="center")
        return
    box = ax.boxplot(series, tick_labels=labels, patch_artist=True, showfliers=False)
    for patch, color in zip(box["boxes"], colors[: len(series)], strict=True):
        patch.set_facecolor(color)
        patch.set_alpha(0.65)
    ax.set_title(title)
    ax.set_ylabel(ylabel)
    ax.grid(True, axis="y", alpha=0.25)

    lines = []
    for idx, values in enumerate(series, start=1):
        median = np.median(values)
        p95 = np.percentile(values, 95)
        lines.append(f"{idx}. median={median:.2f}, p95={p95:.2f}")
    ax.text(
        0.02,
        0.98,
        "\n".join(lines),
        transform=ax.transAxes,
        va="top",
        ha="left",
        fontsize=9,
        bbox={"facecolor": "white", "alpha": 0.85, "edgecolor": "none"},
    )


def _write_notes(path: Path, summary: pd.DataFrame) -> None:
    old_row = summary.iloc[0]
    new_row = summary.iloc[1]
    new_hypo_row = summary.iloc[2]

    text = f"""# Session Mode Stability Notes

Compared sessions:

- Old baseline: `{old_row['session_id']}` in `{old_row['session_dir']}` at `{old_row['lidar_mode']}`
- Latest session: `{new_row['session_id']}` in `{new_row['session_dir']}` at `{new_row['lidar_mode']}`
- Hypothetical latest mode: same latest GPS motion, but scaled as `{new_hypo_row['lidar_mode']}`

Key takeaway:

- The latest route at `{new_row['lidar_mode']}` moves about `{new_row['distance_per_rev_median_m']:.2f} m` per LiDAR revolution at the median and `{new_row['distance_per_rev_p95_m']:.2f} m` at p95.
- The same exact latest route would move about `{new_hypo_row['distance_per_rev_median_m']:.2f} m` per LiDAR revolution at the median and `{new_hypo_row['distance_per_rev_p95_m']:.2f} m` at p95 if captured at `{new_hypo_row['lidar_mode']}`.
- The latest route at `{new_row['lidar_mode']}` turns about `{new_row['turn_per_rev_median_deg']:.2f} deg` per revolution at the median and `{new_row['turn_per_rev_p95_deg']:.2f} deg` at p95.
- The same route would turn about `{new_hypo_row['turn_per_rev_median_deg']:.2f} deg` per revolution at the median and `{new_hypo_row['turn_per_rev_p95_deg']:.2f} deg` at p95 at `{new_hypo_row['lidar_mode']}`.

Interpretation:

- Higher motion during a single LiDAR revolution means more intra-scan distortion pressure.
- `4096x5` doubles the revolution time compared with `1024x10`, so the same physical route accumulates about 2x the within-scan motion.
- That is why these plots are better at explaining mapping instability than raw point density alone.

IMU note:

- I checked the raw Ouster IMU packets in these saved pcaps through the current SDK packet API.
- The packets were present, but their decoded motion fields were effectively zero in this environment.
- Because of that, these plots use recorded GPS speed plus GPS-derived yaw-rate as the trustworthy motion proxy.
"""
    path.write_text(text)


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--old-session-id", default=DEFAULT_OLD_SESSION_ID)
    parser.add_argument("--new-session-id", default=DEFAULT_NEW_SESSION_ID)
    parser.add_argument(
        "--old-label",
        default="Old baseline",
        help="Short label shown in the plots for the old session.",
    )
    parser.add_argument(
        "--new-label",
        default="Latest route",
        help="Short label shown in the plots for the new session.",
    )
    parser.add_argument(
        "--old-mode",
        default=None,
        help="Override lidar mode if it cannot be read from metadata.",
    )
    parser.add_argument(
        "--new-mode",
        default=None,
        help="Override lidar mode if it cannot be read from metadata.",
    )
    parser.add_argument(
        "--hypothetical-mode",
        default=DEFAULT_HYPOTHETICAL_MODE,
        help="Mode used for the 'same latest route if captured differently' comparison.",
    )
    parser.add_argument(
        "--gps-outlier-max-speed-mps",
        type=float,
        default=20.0,
        help="Maximum GPS speed kept by the outlier filter.",
    )
    parser.add_argument(
        "--output-dir",
        default=None,
        help="Directory where plots and summary files will be written.",
    )
    return parser


def main() -> int:
    parser = build_parser()
    args = parser.parse_args()

    output_dir = Path(args.output_dir) if args.output_dir else (
        REPO_ROOT / "test_outputs" / f"mode_compare_{args.old_session_id}_vs_{args.new_session_id}"
    )
    output_dir.mkdir(parents=True, exist_ok=True)

    old_session = _load_session_motion(
        session_id=args.old_session_id,
        label=args.old_label,
        fallback_mode=args.old_mode,
        max_speed_mps=args.gps_outlier_max_speed_mps,
    )
    new_session = _load_session_motion(
        session_id=args.new_session_id,
        label=args.new_label,
        fallback_mode=args.new_mode,
        max_speed_mps=args.gps_outlier_max_speed_mps,
    )
    new_hypothetical = _make_variant(
        new_session,
        label=f"{args.new_label} (same GPS path, hypothetical)",
        lidar_mode=args.hypothetical_mode,
    )

    summary = _summary_rows(old_session, new_session, new_hypothetical)
    summary_csv = output_dir / "mode_motion_summary.csv"
    summary.to_csv(summary_csv, index=False)

    notes_md = output_dir / "mode_motion_notes.md"
    _write_notes(notes_md, summary)

    fig, axes = plt.subplots(2, 2, figsize=(16, 10), constrained_layout=True)
    _plot_speed(axes[0, 0], [old_session, new_session])
    _plot_turn_rate(axes[0, 1], [old_session, new_session])
    _boxplot(
        axes[1, 0],
        [old_session, new_session, new_hypothetical],
        values_key="distance_per_rev_m",
        title="Distance Traveled During One LiDAR Revolution",
        ylabel="Meters per revolution",
    )
    _boxplot(
        axes[1, 1],
        [old_session, new_session, new_hypothetical],
        values_key="turn_per_rev_deg",
        title="Rotation During One LiDAR Revolution",
        ylabel="Degrees per revolution",
    )

    fig.suptitle(
        (
            f"Mobile Mapping Motion Comparison\n"
            f"{old_session.session_id} ({old_session.lidar_mode}) vs "
            f"{new_session.session_id} ({new_session.lidar_mode})"
        ),
        fontsize=16,
    )
    plot_path = output_dir / "mode_motion_comparison.png"
    fig.savefig(plot_path, dpi=180)
    plt.close(fig)

    print(f"Plot written to:    {plot_path}")
    print(f"Summary written to: {summary_csv}")
    print(f"Notes written to:   {notes_md}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
