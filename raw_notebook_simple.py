"""
Simple, focused tools to explore yaw (azimuth) detectability from raw IMU data.

Each function roughly maps to a former notebook cell, but the analysis is
intentionally minimal: a few computations and simple plots/tables.

Conventions assumed:
- Sensor axes: West (x), Up (y), South (z)  [W, U, S]
- Accelerometer units in the CSV: mg (milli-g)
- Gyroscope units in the CSV: mdps (millideg/s)

Notes:
- You cannot get absolute yaw from accelerometer alone, but if there is a small
  tilt, the horizontal projection of gravity rotates with yaw. We exploit this
  to estimate relative yaw change across two windows (before vs after change).
- Gyro-based integration around Up (y) provides a reference for yaw change.
"""

from __future__ import annotations

import math
from typing import Dict, List, Tuple, Optional

import numpy as np
import pandas as pd
import matplotlib.pyplot as plt


# --------------------------- Utility helpers ---------------------------

def _wrap_angle_deg(angle_deg: float) -> float:
    """Wrap any angle in degrees to [-180, 180)."""
    wrapped = (angle_deg + 180.0) % 360.0 - 180.0
    return wrapped


def _estimate_sample_rate_hz(timestamps: np.ndarray) -> float:
    if len(timestamps) < 2:
        return 0.0
    diffs = np.diff(timestamps)
    diffs = diffs[diffs > 0]
    if len(diffs) == 0:
        return 0.0
    return float(np.median(diffs)) ** -1


def _safe_numeric(df: pd.DataFrame, cols: List[str]) -> pd.DataFrame:
    for c in cols:
        if c in df.columns:
            df[c] = pd.to_numeric(df[c], errors="coerce")
    return df


def _vector_norm3(ax: float, ay: float, az: float) -> float:
    return float(math.sqrt(ax * ax + ay * ay + az * az))


# --------------------------- Cell-like functions ---------------------------

def cell_01_default_test_config() -> Dict:
    """Return the default test configuration used to segment and label tests."""
    return {
        "sensor_specs": {
            "accel_range": 2,       # g
            "gyro_range": 2000,     # mdps
            "odr": 104,             # Hz
            "noise_floor": 577e-6,  # g
            "imu_offset": 6.2       # degrees
        },
        "initial_conditions": {
            "theta_init": 1.9,      # degrees (tilt)
            "phi_init": 0.0,        # degrees (azimuth)
        },
        "test_sequence": [
            {"test_id": 0, "duration": 60, "change_time": 20, "description": "Baseline - no movement", "power_state": "on",  "phi_change": 0,  "expected_detectable": False},
            {"test_id": 1, "duration": 60, "change_time": 20, "description": "5° phi rotation (power off)",      "power_state": "off", "phi_change": 5,  "expected_detectable": True},
            {"test_id": 2, "duration": 60, "change_time": 20, "description": "Return to baseline (power on)",     "power_state": "on",  "phi_change": 0,  "expected_detectable": False},
            {"test_id": 3, "duration": 60, "change_time": 20, "description": "5° phi rotation (power on)",       "power_state": "on",  "phi_change": 5,  "expected_detectable": True},
            {"test_id": 4, "duration": 30, "change_time": 15, "description": "10° phi rotation (power on)",      "power_state": "on",  "phi_change": 10, "expected_detectable": True},
            {"test_id": 5, "duration": 30, "change_time": 15, "description": "10° phi rotation (power off)",     "power_state": "off", "phi_change": 10, "expected_detectable": True},
            {"test_id": 6, "duration": 30, "change_time": 15, "description": "15° phi rotation (power off)",     "power_state": "off", "phi_change": 15, "expected_detectable": True},
            {"test_id": 7, "duration": 30, "change_time": 15, "description": "15° phi rotation (power on)",      "power_state": "on",  "phi_change": 15, "expected_detectable": True},
            {"test_id": 8, "duration": 30, "change_time": 15, "description": "5° phi rotation (power off)",      "power_state": "off", "phi_change": 5,  "expected_detectable": True},
            {"test_id": 9, "duration": 30, "change_time": 15, "description": "5° phi rotation (power on)",       "power_state": "on",  "phi_change": 5,  "expected_detectable": True},
            {"test_id": 10, "duration": 30, "change_time": 15, "description": "4° phi rotation (off→on)",         "power_state": "off_then_on", "phi_change": 4, "expected_detectable": True},
            {"test_id": 11, "duration": 30, "change_time": 15, "description": "3° phi rotation (off→on)",         "power_state": "off_then_on", "phi_change": 3, "expected_detectable": False},
            {"test_id": 12, "duration": 30, "change_time": 15, "description": "2° phi rotation (off→on)",         "power_state": "off_then_on", "phi_change": 2, "expected_detectable": False},
        ],
    }


def cell_02_load_data(file_path: str = "raw_data_output.csv") -> pd.DataFrame:
    """Load raw IMU CSV and coerce numeric columns.

    Expects columns: timestamp, accel_x, accel_y, accel_z, gyro_x, gyro_y, gyro_z
    - timestamp: seconds
    - accel_*: mg
    - gyro_*: mdps
    """
    df = pd.read_csv(file_path)
    needed = [
        "timestamp",
        "accel_x", "accel_y", "accel_z",
        "gyro_x", "gyro_y", "gyro_z",
    ]
    missing = [c for c in needed if c not in df.columns]
    if missing:
        raise ValueError(f"Missing columns in CSV: {missing}")

    df = _safe_numeric(df, needed)
    df = df.dropna(subset=["timestamp"]).copy()
    df = df.sort_values("timestamp").reset_index(drop=True)
    return df


def cell_03_segment_tests(
    df: pd.DataFrame,
    config: Dict,
    gap_threshold_s: float = 10.0,
) -> List[Dict]:
    """Split the DataFrame into test segments using large timestamp gaps.

    If fewer segments than tests exist, the last segment is divided proportionally
    by the expected test durations.
    """
    timestamps = df["timestamp"].to_numpy()
    gaps = np.where(np.diff(timestamps) > gap_threshold_s)[0]

    segment_bounds: List[Tuple[int, int]] = []
    prev = 0
    for g in gaps:
        segment_bounds.append((prev, g + 1))
        prev = g + 1
    segment_bounds.append((prev, len(df)))

    tests_meta = config.get("test_sequence", [])
    segments: List[Dict] = []

    # If segments count matches tests, map 1:1
    if len(segment_bounds) == len(tests_meta):
        for i, (s, e) in enumerate(segment_bounds):
            seg = tests_meta[i].copy()
            seg_data = df.iloc[s:e].copy()
            seg["data"] = seg_data
            seg["start_idx"], seg["end_idx"] = int(s), int(e)
            seg["segment_start_time"] = float(seg_data["timestamp"].iloc[0])
            seg["segment_end_time"] = float(seg_data["timestamp"].iloc[-1])
            seg["change_timestamp"] = seg["segment_start_time"] + float(seg["change_time"])
            segments.append(seg)
        return segments

    # Otherwise, try to proportionally slice the last/only segment(s)
    if len(segment_bounds) == 1 and len(tests_meta) > 1:
        s, e = segment_bounds[0]
        seg_all = df.iloc[s:e].copy()
        t0 = float(seg_all["timestamp"].iloc[0])
        t1 = float(seg_all["timestamp"].iloc[-1])
        total_time = t1 - t0 if t1 > t0 else 1.0

        durations = [tm["duration"] for tm in tests_meta]
        cumulative = np.cumsum([0.0] + [d / sum(durations) * total_time for d in durations])

        for i, tm in enumerate(tests_meta):
            start_t = t0 + cumulative[i]
            end_t = t0 + cumulative[i + 1]
            seg_data = seg_all[(seg_all["timestamp"] >= start_t) & (seg_all["timestamp"] < end_t)].copy()
            if seg_data.empty:
                continue
            seg = tm.copy()
            seg["data"] = seg_data
            seg["start_idx"] = int(seg_data.index[0])
            seg["end_idx"] = int(seg_data.index[-1]) + 1
            seg["segment_start_time"] = float(seg_data["timestamp"].iloc[0])
            seg["segment_end_time"] = float(seg_data["timestamp"].iloc[-1])
            seg["change_timestamp"] = seg["segment_start_time"] + float(seg["change_time"])
            segments.append(seg)
        return segments

    # Fallback: map available segments to tests sequentially
    for i, (s, e) in enumerate(segment_bounds):
        seg = tests_meta[i].copy() if i < len(tests_meta) else {"test_id": i}
        seg_data = df.iloc[s:e].copy()
        seg["data"] = seg_data
        seg["start_idx"], seg["end_idx"] = int(s), int(e)
        seg["segment_start_time"] = float(seg_data["timestamp"].iloc[0])
        seg["segment_end_time"] = float(seg_data["timestamp"].iloc[-1])
        if "change_time" in seg:
            seg["change_timestamp"] = seg["segment_start_time"] + float(seg["change_time"])
        segments.append(seg)
    return segments


def cell_04_orientation_quickcheck(df: pd.DataFrame) -> Dict[str, float]:
    """Quick orientation sanity check using mean accelerometer (gravity at rest).

    Returns basic stats useful to confirm W-U-S expectations and tilt magnitude.
    """
    ax_mg = float(df["accel_x"].mean())
    ay_mg = float(df["accel_y"].mean())
    az_mg = float(df["accel_z"].mean())

    g_mg = _vector_norm3(ax_mg, ay_mg, az_mg)
    horizontal_mg = math.sqrt(ax_mg * ax_mg + az_mg * az_mg)
    tilt_rad = math.atan2(horizontal_mg, abs(ay_mg)) if (abs(ay_mg) + horizontal_mg) > 0 else 0.0
    tilt_deg = math.degrees(tilt_rad)

    info = {
        "mean_ax_mg": ax_mg,
        "mean_ay_mg": ay_mg,
        "mean_az_mg": az_mg,
        "mean_g_mg": g_mg,
        "mean_horizontal_mg": horizontal_mg,
        "tilt_deg_est": tilt_deg,
    }
    print(
        f"Mean accel (mg): ax={ax_mg:.1f}, ay={ay_mg:.1f}, az={az_mg:.1f} | "
        f"|g|≈{g_mg:.1f} mg, horizontal≈{horizontal_mg:.1f} mg, tilt≈{tilt_deg:.2f}°"
    )
    return info


def cell_05_accel_yaw_change(
    segment: Dict,
    window_s: float = 5.0,
) -> Dict[str, float]:
    """Estimate yaw change (deg) from accelerometer via horizontal gravity rotation.

    We compute the mean accelerometer vector in two short windows around the
    change timestamp, project each onto the horizontal plane (x-z), and take the
    difference of their angles. Only the difference matters, not absolute yaw.
    """
    data = segment["data"]
    change_ts = float(segment["change_timestamp"]) if "change_timestamp" in segment else float(data["timestamp"].iloc[len(data)//2])

    # Window masks
    t = data["timestamp"].to_numpy()
    before_mask = (t >= (change_ts - window_s)) & (t < change_ts)
    after_mask = (t >= change_ts) & (t < (change_ts + window_s))

    before = data.loc[before_mask]
    after = data.loc[after_mask]

    if len(before) < 5 or len(after) < 5:
        return {"status": 0, "delta_phi_accel_deg": float("nan")}

    ax_b = float(before["accel_x"].mean())
    ay_b = float(before["accel_y"].mean())
    az_b = float(before["accel_z"].mean())

    ax_a = float(after["accel_x"].mean())
    ay_a = float(after["accel_y"].mean())
    az_a = float(after["accel_z"].mean())

    # Horizontal projections (x-z plane)
    phi_b = math.degrees(math.atan2(ax_b, az_b))
    phi_a = math.degrees(math.atan2(ax_a, az_a))

    delta_phi = _wrap_angle_deg(phi_a - phi_b)

    # Diagnostics: tilt and horizontal magnitudes
    h_b = math.sqrt(ax_b * ax_b + az_b * az_b)
    h_a = math.sqrt(ax_a * ax_a + az_a * az_a)
    tilt_b = math.degrees(math.atan2(h_b, abs(ay_b))) if (abs(ay_b) + h_b) > 0 else 0.0
    tilt_a = math.degrees(math.atan2(h_a, abs(ay_a))) if (abs(ay_a) + h_a) > 0 else 0.0

    return {
        "status": 1,
        "delta_phi_accel_deg": float(delta_phi),
        "phi_before_deg": float(phi_b),
        "phi_after_deg": float(phi_a),
        "h_before_mg": float(h_b),
        "h_after_mg": float(h_a),
        "tilt_before_deg": float(tilt_b),
        "tilt_after_deg": float(tilt_a),
    }


def cell_06_gyro_yaw_change(
    segment: Dict,
    window_s: float = 5.0,
) -> Dict[str, float]:
    """Integrate gyro around Up (y) to estimate yaw change (deg) after change.

    Integrates gyro_y (mdps -> deg/s) over the window [change_time, change_time+window_s].
    """
    data = segment["data"]
    change_ts = float(segment["change_timestamp"]) if "change_timestamp" in segment else float(data["timestamp"].iloc[len(data)//2])

    sub = data[(data["timestamp"] >= change_ts) & (data["timestamp"] < change_ts + window_s)].copy()
    if len(sub) < 2:
        return {"status": 0, "delta_phi_gyro_deg": float("nan")}

    t = sub["timestamp"].to_numpy()
    rate_deg_s = sub["gyro_y"].to_numpy(dtype=float) / 1000.0  # mdps -> deg/s

    dt = np.diff(t)
    mid = 0.5 * (rate_deg_s[:-1] + rate_deg_s[1:])
    yaw_delta_deg = float(np.sum(mid * dt))

    # For reference, pre-window drift
    pre = data[(data["timestamp"] >= change_ts - window_s) & (data["timestamp"] < change_ts)].copy()
    drift_deg = float("nan")
    if len(pre) >= 2:
        t_pre = pre["timestamp"].to_numpy()
        r_pre = pre["gyro_y"].to_numpy(dtype=float) / 1000.0
        drift_deg = float(np.sum(0.5 * (r_pre[:-1] + r_pre[1:]) * np.diff(t_pre)))

    return {"status": 1, "delta_phi_gyro_deg": yaw_delta_deg, "pre_window_drift_deg": drift_deg}


def cell_07_build_summary(
    segments: List[Dict],
    window_s: float = 5.0,
) -> pd.DataFrame:
    """Compute a concise per-test summary with accel and gyro yaw changes."""
    rows = []
    for seg in segments:
        acc = cell_05_accel_yaw_change(seg, window_s=window_s)
        gyr = cell_06_gyro_yaw_change(seg, window_s=window_s)

        rows.append({
            "test_id": seg.get("test_id"),
            "power_state": seg.get("power_state"),
            "phi_expected_deg": seg.get("phi_change"),
            "delta_phi_accel_deg": acc.get("delta_phi_accel_deg"),
            "delta_phi_gyro_deg": gyr.get("delta_phi_gyro_deg"),
            "tilt_before_deg": acc.get("tilt_before_deg"),
            "tilt_after_deg": acc.get("tilt_after_deg"),
            "h_before_mg": acc.get("h_before_mg"),
            "h_after_mg": acc.get("h_after_mg"),
        })
    df = pd.DataFrame(rows)
    return df


def cell_08_plot_test_minimal(segment: Dict, window_s: float = 5.0) -> None:
    """Minimal 2-panel plot: gyro_y and accel-based horizontal angle vs time."""
    data = segment["data"]
    change_ts = float(segment["change_timestamp"]) if "change_timestamp" in segment else float(data["timestamp"].iloc[len(data)//2])

    t = data["timestamp"].to_numpy()
    gyro_y_mdps = data["gyro_y"].to_numpy(dtype=float)

    # Accel-based instantaneous horizontal angle (noisy, but simple)
    ax = data["accel_x"].to_numpy(dtype=float)
    az = data["accel_z"].to_numpy(dtype=float)
    phi_inst_deg = np.degrees(np.arctan2(ax, az))

    # Rolling average with ~0.5 s window for readability
    fs = _estimate_sample_rate_hz(t)
    win = max(1, int(round(0.5 * fs))) if fs > 0 else 1
    phi_roll = pd.Series(phi_inst_deg).rolling(win, center=True, min_periods=max(1, win // 2)).mean().to_numpy()

    fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(10, 6), sharex=True)

    ax1.plot(t, gyro_y_mdps, color="tab:blue", linewidth=1.0)
    ax1.axvline(change_ts, color="red", linestyle="--", linewidth=1.0)
    ax1.set_ylabel("gyro_y (mdps)")
    ax1.grid(True, alpha=0.3)

    ax2.plot(t, phi_roll, color="tab:green", linewidth=1.0, label="accel yaw (deg)")
    ax2.axvline(change_ts, color="red", linestyle="--", linewidth=1.0)
    ax2.set_xlabel("time (s)")
    ax2.set_ylabel("accel yaw est (deg)")
    ax2.grid(True, alpha=0.3)

    fig.suptitle(f"Test {segment.get('test_id')} — {segment.get('description', '')}")
    plt.tight_layout()
    plt.show()


def cell_09_compare_power_states(summary_df: pd.DataFrame) -> None:
    """Print a concise table and draw simple scatter comparisons by power state."""
    if summary_df.empty:
        print("No summary data.")
        return

    display_cols = [
        "test_id", "power_state", "phi_expected_deg",
        "delta_phi_gyro_deg", "delta_phi_accel_deg",
        "tilt_before_deg",
    ]
    print(summary_df[display_cols].round(2).to_string(index=False))

    # Scatter: expected vs measured (gyro and accel), colored by power state
    fig, (ax1, ax2) = plt.subplots(1, 2, figsize=(10, 4))
    for state, color in [("on", "tab:green"), ("off", "tab:red"), ("off_then_on", "tab:orange")]:
        sub = summary_df[summary_df["power_state"] == state]
        if sub.empty:
            continue
        ax1.scatter(sub["phi_expected_deg"], sub["delta_phi_gyro_deg"], c=color, label=state, alpha=0.8)
        ax2.scatter(sub["phi_expected_deg"], sub["delta_phi_accel_deg"], c=color, label=state, alpha=0.8)

    for ax in (ax1, ax2):
        lim = max(1.0, float(np.nanmax(np.abs(summary_df[["phi_expected_deg", "delta_phi_gyro_deg", "delta_phi_accel_deg"]]))))
        ax.plot([-lim, lim], [-lim, lim], "k--", linewidth=1.0, alpha=0.5)
        ax.set_xlabel("expected φ (deg)")
        ax.set_ylabel("measured φ (deg)")
        ax.grid(True, alpha=0.3)
        ax.legend()

    ax1.set_title("Gyro-based")
    ax2.set_title("Accel-based")
    plt.tight_layout()
    plt.show()


def cell_10_run(
    file_path: str = "raw_data_output.csv",
    gap_threshold_s: float = 10.0,
    window_s: float = 5.0,
    plot_first_n: int = 3,
) -> pd.DataFrame:
    """End-to-end: load, segment, quick-check, compute summary, show minimal plots.

    Returns the summary DataFrame.
    """
    cfg = cell_01_default_test_config()
    df = cell_02_load_data(file_path)
    segments = cell_03_segment_tests(df, cfg, gap_threshold_s=gap_threshold_s)
    print(f"Identified {len(segments)} segment(s).")
    cell_04_orientation_quickcheck(df)

    summary = cell_07_build_summary(segments, window_s=window_s)
    print("\nPer-test summary (deg):")
    cell_09_compare_power_states(summary)

    # Plot a few examples
    for seg in segments[: max(0, int(plot_first_n))]:
        cell_08_plot_test_minimal(seg, window_s=window_s)

    return summary


if __name__ == "__main__":
    # Default runner; adjust parameters as needed
    try:
        _ = cell_10_run()
    except Exception as exc:
        print(f"Error: {exc}")

