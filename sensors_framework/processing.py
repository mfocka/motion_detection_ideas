from __future__ import annotations

import numpy as np
from scipy.signal import butter, filtfilt

G_TO_MS2: float = 9.80665
MG_TO_MS2: float = G_TO_MS2 / 1000.0
DEG_TO_RAD: float = np.pi / 180.0


def _butter_lowpass(cutoff_hz: float, fs_hz: float, order: int = 2):
    nyquist_hz = 0.5 * fs_hz
    normalized_cutoff = min(max(cutoff_hz / nyquist_hz, 1e-6), 0.999999)
    b, a = butter(order, normalized_cutoff, btype="low", analog=False)
    return b, a


def preprocess_imu(
    accel_mg: np.ndarray,
    gyro_mdps: np.ndarray,
    lp_cutoff_hz: float = 5.0,
    fs_hz: float = 104.0,
    apply_filter: bool = True,
):
    """
    Convert raw IMU arrays to SI units and optionally apply a low-pass filter.

    Parameters
    ----------
    accel_mg : ndarray shape (N, 3)
        Accelerometer in milli-g (mg).
    gyro_mdps : ndarray shape (N, 3)
        Gyroscope in milli-deg/s (mdps).
    lp_cutoff_hz : float
        Low-pass cutoff frequency in Hz.
    fs_hz : float
        Sampling rate in Hz.
    apply_filter : bool
        Whether to apply a zero-phase low-pass filter.

    Returns
    -------
    accel_ms2 : ndarray shape (N, 3)
        Accelerometer in m/s^2 (optionally filtered).
    gyro_rps : ndarray shape (N, 3)
        Gyroscope in rad/s (optionally filtered).
    """

    accel_ms2 = np.asarray(accel_mg, dtype=float) * MG_TO_MS2
    gyro_rps = np.asarray(gyro_mdps, dtype=float) * (DEG_TO_RAD * 1000.0)

    if not apply_filter:
        return accel_ms2, gyro_rps

    if accel_ms2.ndim != 2 or accel_ms2.shape[1] != 3:
        raise ValueError("accel_mg must be shape (N,3)")
    if gyro_rps.ndim != 2 or gyro_rps.shape[1] != 3:
        raise ValueError("gyro_mdps must be shape (N,3)")

    b, a = _butter_lowpass(lp_cutoff_hz, fs_hz, order=2)

    accel_filt = np.empty_like(accel_ms2)
    gyro_filt = np.empty_like(gyro_rps)
    for axis in range(3):
        accel_filt[:, axis] = filtfilt(b, a, accel_ms2[:, axis], method="gust")
        gyro_filt[:, axis] = filtfilt(b, a, gyro_rps[:, axis], method="gust")

    return accel_filt, gyro_filt

