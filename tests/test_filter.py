"""Self-check: the Python filter reproduces the recorded MATLAB output (course-style process model)
and the corrected process model is consistent with the measurement model. Run: python tests/test_filter.py"""
import sys
from pathlib import Path
sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'src' / 'python'))
import numpy as np
from orientation_ekf import load, run_filter, default_config
from orientation_ekf.quaternion import Qq, dQqdq, normalize

# 1. same output as MATLAB on a recording (first 3000 samples keep it fast)
m = load('C_gyro_acc_gated')
for k in ('t', 'acc', 'gyr', 'mag'):
    setattr(m, k, getattr(m, k)[..., :3000])
r = run_filter(m, default_config(use_mag=False))
assert np.abs(r['x'] - m.xhat_matlab[:, :3000]).max() < 1e-6

# 2. analytic Jacobian dQ/dq matches finite differences
q = normalize(np.array([.7, .1, -.3, .4])); v = np.array([.3, -1., 2.])
J = np.column_stack([d @ v for d in dQqdq(q)])
Jn = np.column_stack([(Qq(q + 1e-6 * e) @ v - Qq(q - 1e-6 * e) @ v) / 2e-6 for e in np.eye(4)])
assert np.abs(J - Jn).max() < 1e-6

print('ok')
