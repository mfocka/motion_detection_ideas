import numpy as np
from dataclasses import dataclass, field
from collections import deque
import matplotlib.pyplot as plt

# --------------------------
# Global Parameters
# --------------------------
FS = 100                 # Sampling frequency [Hz]
DT = 1.0 / FS            # Sample period [s]
ACC_MEAN_MG = np.array([-40, -120, -800])   # mg (static bias)
GYRO_MEAN_DPS = np.array([12, -15, 8])      # deg/s (static bias)
ACC_NOISE_MG_BASE = 50   # 1-sigma accelerometer noise, mg
GYRO_NOISE_DPS_BASE = 20 # 1-sigma gyro noise, deg/s

# --------------------------
# Data classes
# --------------------------
@dataclass
class SimParams:
    """Parameters that shape the synthetic IMU dataset."""
    duration_static: float = 60.0   # seconds before event
    duration_event: float = 5.0     # seconds during motion
    duration_hold: float = 120.0    # seconds after motion
    acc_event_offset_mg: np.ndarray = field(default_factory=lambda: np.array([400, 400, 500]))
    gyro_event_rate_dps: np.ndarray = field(default_factory=lambda: np.array([100, 120, -80]))

# --------------------------
# Simulation helpers
# --------------------------

def simulate_imu(params: SimParams = SimParams(),
                 acc_noise_mg: float = ACC_NOISE_MG_BASE,
                 gyro_noise_dps: float = GYRO_NOISE_DPS_BASE,
                 seed: int | None = None):
    """Generate synthetic IMU data and return (t, acc, gyro, event_start, event_end)."""
    if seed is not None:
        np.random.seed(seed)

    n_static = int(params.duration_static * FS)
    n_event = int(params.duration_event * FS)
    n_hold = int(params.duration_hold * FS)

    # Static period
    acc_static = ACC_MEAN_MG + np.random.randn(n_static, 3) * acc_noise_mg
    gyro_static = GYRO_MEAN_DPS + np.random.randn(n_static, 3) * gyro_noise_dps

    # Event (abrupt change)
    acc_event = (ACC_MEAN_MG + params.acc_event_offset_mg +
                 np.random.randn(n_event, 3) * acc_noise_mg)
    gyro_event = (params.gyro_event_rate_dps +
                  np.random.randn(n_event, 3) * gyro_noise_dps)

    # Hold (new orientation)
    acc_new_mean = ACC_MEAN_MG + params.acc_event_offset_mg
    acc_hold = acc_new_mean + np.random.randn(n_hold, 3) * acc_noise_mg
    gyro_hold = GYRO_MEAN_DPS + np.random.randn(n_hold, 3) * gyro_noise_dps

    acc = np.vstack((acc_static, acc_event, acc_hold))
    gyro = np.vstack((gyro_static, gyro_event, gyro_hold))
    t = np.arange(acc.shape[0]) * DT

    event_start = n_static
    event_end = n_static + n_event
    return t, acc, gyro, event_start, event_end

# --------------------------
# Basic signal processing
# --------------------------

def lowpass(data: np.ndarray, alpha: float) -> np.ndarray:
    """Simple exponential low-pass filter (in-place friendly)."""
    y = np.empty_like(data)
    y[0] = data[0]
    for i in range(1, len(data)):
        y[i] = alpha * data[i] + (1.0 - alpha) * y[i - 1]
    return y

# --------------------------
# Orientation estimation
# --------------------------

def tilt_angles(acc_mg: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
    """Compute pitch and roll in degrees from accelerometer only."""
    ax, ay, az = acc_mg.T
    ax_g, ay_g, az_g = ax / 1000.0, ay / 1000.0, az / 1000.0
    pitch = np.degrees(np.arctan2(-ax_g, np.sqrt(ay_g**2 + az_g**2)))
    roll = np.degrees(np.arctan2(ay_g, az_g))
    return pitch, roll


def complementary_filter(acc_mg: np.ndarray, gyro_dps: np.ndarray,
                         alpha: float = 0.98, dt: float = DT) -> tuple[np.ndarray, np.ndarray]:
    """First-order complementary filter for pitch & roll fusion (gyro + acc)."""
    acc_pitch, acc_roll = tilt_angles(acc_mg)
    n = acc_pitch.size
    pitch = np.zeros(n)
    roll = np.zeros(n)
    pitch[0] = acc_pitch[0]
    roll[0] = acc_roll[0]
    for i in range(1, n):
        gx, gy, _ = gyro_dps[i]
        roll_gyro = roll[i-1] + gx * dt
        pitch_gyro = pitch[i-1] + gy * dt
        roll[i] = alpha * roll_gyro + (1.0 - alpha) * acc_roll[i]
        pitch[i] = alpha * pitch_gyro + (1.0 - alpha) * acc_pitch[i]
    return pitch, roll

# --------------------------
# Abrupt change detection
# --------------------------

def detect_abrupt_changes(pitch: np.ndarray, roll: np.ndarray,
                          threshold_deg: float = 5.0,
                          window_s: float = 2.0) -> np.ndarray:
    """Binary array indicating samples that exceed the adaptive baseline threshold."""
    win = int(window_s * FS)
    dq_pitch, dq_roll = deque(maxlen=win), deque(maxlen=win)
    evt = np.zeros_like(pitch, dtype=bool)
    for i in range(pitch.size):
        dq_pitch.append(pitch[i])
        dq_roll.append(roll[i])
        if len(dq_pitch) < win:
            continue  # build baseline
        bp, br = np.mean(dq_pitch), np.mean(dq_roll)
        if abs(pitch[i] - bp) > threshold_deg or abs(roll[i] - br) > threshold_deg:
            evt[i] = True
    return evt

# --------------------------
# Calibration helpers
# --------------------------

def calibrate(acc_mg: np.ndarray, gyro_dps: np.ndarray, n_samples: int = 500):
    """Simple static calibration over the first `n_samples` readings."""
    window_acc = acc_mg[:n_samples]
    window_gyro = gyro_dps[:n_samples]
    acc_bias = np.mean(window_acc, axis=0)
    gyro_bias = np.mean(window_gyro, axis=0)
    pitch0, roll0 = tilt_angles(acc_bias.reshape(1, 3))
    return {
        'acc_bias': acc_bias,
        'gyro_bias': gyro_bias,
        'pitch0': float(pitch0[0]),
        'roll0': float(roll0[0]),
    }


def save_calibration(calib: dict, path: str):
    """Persist calibration dictionary as JSON."""
    import json, pathlib
    serialisable = {k: (v.tolist() if isinstance(v, np.ndarray) else v) for k, v in calib.items()}
    pathlib.Path(path).write_text(json.dumps(serialisable))


def load_calibration(path: str) -> dict:
    """Load calibration dictionary from JSON file."""
    import json, pathlib
    data = json.loads(pathlib.Path(path).read_text())
    if 'acc_bias' in data:
        data['acc_bias'] = np.array(data['acc_bias'])
    if 'gyro_bias' in data:
        data['gyro_bias'] = np.array(data['gyro_bias'])
    return data

# --------------------------
# Evaluation utility
# --------------------------

def run_trials(n_trials: int = 20,
               acc_noise_mg: float = ACC_NOISE_MG_BASE,
               gyro_noise_dps: float = GYRO_NOISE_DPS_BASE,
               seed: int | None = 42):
    """Monte-Carlo evaluation of detection performance."""
    rng = np.random.default_rng(seed)
    latencies = []
    fp_rates = []
    for _ in range(n_trials):
        trial_seed = rng.integers(0, 1e6)
        t, acc, gyro, es, ee = simulate_imu(seed=int(trial_seed))
        pitch, roll = complementary_filter(acc, gyro)
        pitch_lp = lowpass(pitch, 0.1)
        roll_lp = lowpass(roll, 0.1)
        abrupt = detect_abrupt_changes(pitch_lp, roll_lp)
        # latency: first detection during event window
        idx_after_event = np.where(abrupt[es:ee])[0]
        if idx_after_event.size:
            latencies.append(idx_after_event[0] * DT)
        # false positives per hour before event
        fp_rates.append(np.sum(abrupt[:es]) / (es * DT / 3600))
    det_rate = len(latencies) / n_trials
    mean_latency = float(np.mean(latencies)) if latencies else float('nan')
    median_fp = float(np.median(fp_rates))
    return det_rate, mean_latency, median_fp

# --------------------------
# Plotting utilities
# --------------------------

def plot_simulation(t: np.ndarray, acc: np.ndarray, gyro: np.ndarray,
                    pitch: np.ndarray, roll: np.ndarray,
                    abrupt: np.ndarray, event_start: int, event_end: int,
                    save_path: str | None = None):
    """Generate summary plots showing raw data, estimated orientation, and detections."""
    fig, axes = plt.subplots(3, 1, figsize=(12, 8), sharex=True)

    # Accelerometer
    axes[0].plot(t, acc)
    axes[0].set_ylabel('Accel [mg]')
    axes[0].legend(['Ax', 'Ay', 'Az'])

    # Gyroscope
    axes[1].plot(t, gyro)
    axes[1].set_ylabel('Gyro [dps]')
    axes[1].legend(['Gx', 'Gy', 'Gz'])

    # Orientation + detections
    axes[2].plot(t, pitch, label='Pitch')
    axes[2].plot(t, roll, label='Roll')
    # mark detections
    axes[2].scatter(t[abrupt], pitch[abrupt], c='r', s=10, label='Detections')
    # true event window shading
    axes[2].axvspan(t[event_start], t[event_end-1], color='orange', alpha=0.2, label='True event')
    axes[2].set_ylabel('Angle [deg]')
    axes[2].set_xlabel('Time [s]')
    axes[2].legend()

    fig.suptitle('IMU Simulation, Orientation Estimation and Abrupt Change Detection')
    plt.tight_layout()
    if save_path is not None:
        fig.savefig(save_path, dpi=300)
    plt.show()


def apply_calibration(acc_mg: np.ndarray, gyro_dps: np.ndarray, calib: dict):
    """Return bias-compensated sensor arrays using a calibration dict."""
    acc_corr = acc_mg - calib.get('acc_bias', 0)
    gyro_corr = gyro_dps - calib.get('gyro_bias', 0)
    return acc_corr, gyro_corr


def demo_plot(threshold_deg: float = 5.0):
    """Convenience wrapper to run one simulation and plot the results."""
    t, acc, gyro, es, ee = simulate_imu()
    pitch, roll = complementary_filter(acc, gyro)
    pitch_lp = lowpass(pitch, 0.1)
    roll_lp = lowpass(roll, 0.1)
    abrupt = detect_abrupt_changes(pitch_lp, roll_lp, threshold_deg=threshold_deg)
    plot_simulation(t, acc, gyro, pitch_lp, roll_lp, abrupt, es, ee)

# --------------------------
# Quick self-test
# --------------------------
if __name__ == "__main__":
    det, lat, fp = run_trials(n_trials=30)
    print("==== Quick Evaluation ====")
    print(f"Detection rate : {det*100:.1f}%")
    print(f"Mean latency   : {lat:.2f} s")
    print(f"Median FP/h    : {fp:.2f}\n")
    print("Module executed successfully.")
    # Show example plots
    demo_plot()
