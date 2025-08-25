from __future__ import annotations

from dataclasses import dataclass
from typing import List, Dict

import numpy as np


@dataclass
class OrientationEvent:
    index: int
    timestamp_s: float
    delta_roll: float
    delta_pitch: float
    delta_yaw: float
    persistent: bool


def detect_changes(
    euler_rpy: np.ndarray,
    az_thresh_deg: float = 5.0,
    alt_thresh_deg: float = 5.0,
    T_detection_s: float = 2.0,
    T_validation_s: float = 600.0,
    fs_hz: float = 104.0,
    smoothing_window_s: float = 0.5,
) -> List[OrientationEvent]:
    """
    Detect abrupt orientation changes with thresholding and persistence check.

    euler_rpy: ndarray (N,3) in radians [roll, pitch, yaw]
    Returns list of OrientationEvent.
    """
    if euler_rpy.ndim != 2 or euler_rpy.shape[1] != 3:
        raise ValueError("euler_rpy must be shape (N,3)")

    deg = 180.0 / np.pi
    euler_deg = euler_rpy * deg

    # Smooth angles to ignore vibration
    window = max(int(round(smoothing_window_s * fs_hz)), 1)
    if window > 1:
        kernel = np.ones(window) / window
        euler_deg_smooth = np.vstack([
            np.convolve(euler_deg[:, i], kernel, mode="same") for i in range(3)
        ]).T
    else:
        euler_deg_smooth = euler_deg

    # Compute deltas over detection window
    det_n = max(int(round(T_detection_s * fs_hz)), 1)
    N = euler_deg_smooth.shape[0]
    events: List[OrientationEvent] = []

    for t in range(det_n, N):
        delta = euler_deg_smooth[t] - euler_deg_smooth[t - det_n]
        delta_roll, delta_pitch, delta_yaw = delta

        az_ok = abs(delta_yaw) >= az_thresh_deg
        alt_ok = abs(delta_pitch) >= alt_thresh_deg
        if not (az_ok or alt_ok):
            continue

        # Persistence: orientation must remain near the new value for T_validation
        val_n = max(int(round(T_validation_s * fs_hz)), 1)
        end_idx = min(t + val_n, N - 1)
        new_orientation = euler_deg_smooth[t]
        segment = euler_deg_smooth[t:end_idx]
        # Check if segment stays within 50% of the detected change relative to prior
        prior = euler_deg_smooth[t - det_n]
        target_delta = new_orientation - prior
        tol = 0.5 * np.abs(target_delta) + 1.0  # add 1 deg slack
        persistent = bool(np.all(np.max(np.abs(segment - new_orientation), axis=0) <= tol))

        events.append(
            OrientationEvent(
                index=t,
                timestamp_s=t / fs_hz,
                delta_roll=float(delta_roll),
                delta_pitch=float(delta_pitch),
                delta_yaw=float(delta_yaw),
                persistent=persistent,
            )
        )

    return events

