from __future__ import annotations

from typing import Dict, List

import numpy as np


def _match_events(pred_times: np.ndarray, gt_times: np.ndarray, tolerance_s: float = 2.0):
    used_gt = np.zeros(len(gt_times), dtype=bool)
    tp = 0
    latencies: List[float] = []
    for t in pred_times:
        diffs = np.abs(gt_times - t)
        idx = int(np.argmin(diffs)) if len(diffs) else 0
        if len(diffs) and diffs[idx] <= tolerance_s and not used_gt[idx]:
            used_gt[idx] = True
            tp += 1
            latencies.append(diffs[idx])
    fp = max(0, len(pred_times) - tp)
    fn = max(0, len(gt_times) - tp)
    return tp, fp, fn, latencies


def evaluate_algorithm(
    euler_rpy: np.ndarray,
    ground_truth: Dict[str, np.ndarray],
    events: List,
    tolerance_s: float = 2.0,
) -> Dict[str, float]:
    """
    Compute precision, recall, F1, latency, and false positives per hour.
    ground_truth: expects dict with key "times_s" array of change times.
    """
    if not len(events):
        pred_times = np.array([], dtype=float)
    else:
        pred_times = np.array([ev.timestamp_s for ev in events if getattr(ev, "persistent", True)])
    gt_times = np.asarray(ground_truth.get("times_s", []), dtype=float)

    tp, fp, fn, latencies = _match_events(pred_times, gt_times, tolerance_s=tolerance_s)

    precision = tp / (tp + fp) if (tp + fp) > 0 else 0.0
    recall = tp / (tp + fn) if (tp + fn) > 0 else 0.0
    f1 = 2 * precision * recall / (precision + recall) if (precision + recall) > 0 else 0.0
    latency = float(np.mean(latencies)) if latencies else float("nan")

    duration_s = len(euler_rpy) / ground_truth.get("fs_hz", 104.0)
    fph = (fp / duration_s) * 3600.0 if duration_s > 0 else float("nan")

    return {
        "precision": float(precision),
        "recall": float(recall),
        "f1": float(f1),
        "latency_s": float(latency),
        "false_positives_per_hour": float(fph),
        "tp": int(tp),
        "fp": int(fp),
        "fn": int(fn),
    }

