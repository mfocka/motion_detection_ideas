from __future__ import annotations

import numpy as np
from ahrs.filters import Complementary as CF
from ahrs.filters import Madgwick, Mahony
from ahrs.common.orientation import q2euler


def estimate_orientation(
    accel_ms2: np.ndarray,
    gyro_rps: np.ndarray,
    method: str = "complementary",
    fs_hz: float = 104.0,
    gain: float = 0.1,
):
    """
    Estimate orientation as Euler angles [roll, pitch, yaw] in radians.
    Returns ndarray shape (N, 3).
    """
    num_samples = accel_ms2.shape[0]
    euler = np.zeros((num_samples, 3), dtype=float)

    method = method.lower()
    if method == "complementary":
        cf = CF(acc=accel_ms2, gyr=gyro_rps, frequency=fs_hz, gain=gain)
        for idx, q in enumerate(cf.Q):
            euler[idx] = q2euler(q)
        return euler

    if method == "madgwick":
        filter_impl = Madgwick(frequency=fs_hz)
        Q = np.tile([1.0, 0.0, 0.0, 0.0], (num_samples, 1))
        for t in range(1, num_samples):
            Q[t] = filter_impl.updateIMU(Q[t - 1], gyr=gyro_rps[t], acc=accel_ms2[t])
            euler[t] = q2euler(Q[t])
        return euler

    if method == "mahony":
        filter_impl = Mahony(frequency=fs_hz)
        Q = np.tile([1.0, 0.0, 0.0, 0.0], (num_samples, 1))
        for t in range(1, num_samples):
            Q[t] = filter_impl.updateIMU(Q[t - 1], gyr=gyro_rps[t], acc=accel_ms2[t])
            euler[t] = q2euler(Q[t])
        return euler

    # EKF placeholder: available in ahrs.filters as EKF but requires setup.
    if method == "ekf":
        from ahrs.filters import EKF
        filter_impl = EKF(frequency=fs_hz)
        Q = np.tile([1.0, 0.0, 0.0, 0.0], (num_samples, 1))
        for t in range(1, num_samples):
            Q[t] = filter_impl.update(Q[t - 1], gyr=gyro_rps[t], acc=accel_ms2[t])
            euler[t] = q2euler(Q[t])
        return euler

    raise ValueError(f"Unknown method: {method}")

