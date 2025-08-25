from __future__ import annotations

import json
from pathlib import Path
from typing import Iterable, Tuple

import numpy as np


def save_baseline(euler_rpy: np.ndarray, path: str = "baseline.json"):
    euler_rpy = np.asarray(euler_rpy)
    baseline = {
        "mean_rpy": euler_rpy.mean(axis=0).tolist(),
        "std_rpy": euler_rpy.std(axis=0).tolist(),
    }
    Path(path).write_text(json.dumps(baseline, indent=2))


def load_baseline(path: str = "baseline.json") -> np.ndarray:
    p = Path(path)
    if not p.exists():
        return np.zeros(3, dtype=float)
    data = json.loads(p.read_text())
    return np.asarray(data.get("mean_rpy", [0.0, 0.0, 0.0]), dtype=float)

