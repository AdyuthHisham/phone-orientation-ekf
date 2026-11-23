"""Quaternion EKF for orientation from gyroscope, accelerometer and magnetometer.

State x = unit quaternion (world -> body), covariance P (4x4).
Per sample, in the same order as the course/live implementation:
    accelerometer update  ->  gyroscope time update  ->  magnetometer update
"""
from dataclasses import dataclass, field, replace
import numpy as np
from .quaternion import Sq, Somega, Sq_wb, Somega_wb, Qq, dQqdq, normalize


@dataclass
class Config:
    g0: np.ndarray            # gravity in the world frame [m/s^2]
    m0: np.ndarray            # magnetic field in the world frame [uT]: [0, horizontal, vertical]
    Ra: np.ndarray            # accelerometer noise covariance
    Rw: np.ndarray            # gyroscope noise covariance (drives process noise via Sq)
    Rm: np.ndarray            # magnetometer noise covariance
    L0: np.ndarray            # initial running mean of the magnetometer (outlier gate)
    acc_gate: tuple = (9.6, 9.95)   # accept acc update only if |a| in this band [m/s^2]; None = off
    mag_alpha: float = 0.6          # AR(1) coefficient of the running mean L_hat
    mag_thr: float = 0.9            # accept mag update if |m - L_hat| < thr [uT]; None = off
    mag_norm_tol: float = None      # extra gate (not in the course version): | |m| - |m0| | < tol
    legacy_process_model: bool = False  # True = course-template Somega/Sq (inconsistent with Q(q); see docs/findings.md)
    use_acc: bool = True
    use_gyr: bool = True
    use_mag: bool = True
    x0: np.ndarray = field(default_factory=lambda: np.array([1.0, 0, 0, 0]))

    def but(self, **kw):
        return replace(self, **kw)


def time_update(x, P, omega, T, Rw, legacy=False):
    S, Sn = (Somega, Sq) if legacy else (Somega_wb, Sq_wb)
    F = np.eye(4) + 0.5 * T * S(omega)
    G = 0.5 * T * Sn(x)
    x = normalize(F @ x)
    return x, F @ P @ F.T + G @ Rw @ G.T


def meas_update(x, P, y, R, v0):
    """EKF update for a sensor that measures y = Q(q) v0 + noise."""
    H = np.column_stack([dQ @ v0 for dQ in dQqdq(x)])
    S = H @ P @ H.T + R
    K = P @ H.T @ np.linalg.inv(S)
    x = x + K @ (y - Qq(x) @ v0)
    return normalize(x), P - K @ S @ K.T


def run_filter(meas, cfg):
    """Run the filter over a recording. `meas` needs t, acc, gyr, mag as (3, N) / (N,) arrays.

    Returns dict: x (4,N), P (N,4,4), acc_ok / mag_ok (N,) bool = update was applied,
    L_hat (3,N) running magnetometer mean.
    """
    N = len(meas.t)
    x, P, L = cfg.x0.copy(), np.eye(4), cfg.L0.copy()
    out = dict(x=np.empty((4, N)), P=np.empty((N, 4, 4)), acc_ok=np.zeros(N, bool),
               mag_ok=np.zeros(N, bool), L_hat=np.empty((3, N)))
    t_prev = meas.t[0]
    m0n = np.linalg.norm(cfg.m0)
    for k in range(N):
        T, t_prev = meas.t[k] - t_prev, meas.t[k]

        a = meas.acc[:, k]
        if cfg.use_acc and not np.isnan(a).any():
            if cfg.acc_gate is None or cfg.acc_gate[0] < np.linalg.norm(a) < cfg.acc_gate[1]:
                x, P = meas_update(x, P, a, cfg.Ra, cfg.g0)
                out['acc_ok'][k] = True

        w = meas.gyr[:, k]
        if cfg.use_gyr and not np.isnan(w).any():
            x, P = time_update(x, P, w, T, cfg.Rw, cfg.legacy_process_model)

        b = meas.mag[:, k]
        if cfg.use_mag and not np.isnan(b).any():
            ok = True
            if cfg.mag_thr is not None:
                L = (1 - cfg.mag_alpha) * L + cfg.mag_alpha * b
                ok = np.linalg.norm(b - L) < cfg.mag_thr
            if cfg.mag_norm_tol is not None:
                ok = ok and abs(np.linalg.norm(b) - m0n) < cfg.mag_norm_tol
            if ok:
                x, P = meas_update(x, P, b, cfg.Rm, cfg.m0)
                out['mag_ok'][k] = True

        out['x'][:, k], out['P'][k], out['L_hat'][:, k] = x, P, L
    return out
