import json
from pathlib import Path
import numpy as np
from scipy.io import loadmat
from .ekf import Config
from .quaternion import conj

ROOT = Path(__file__).resolve().parents[3]
REC = ROOT / 'data' / 'recordings'


def load(name):
    """Load a recording. Returns a namespace with t, acc, gyr, mag, ref, xhat_matlab.

    ref is the phone's own orientation, converted to the filter convention (conjugated).
    It is a *reference*, not ground truth.
    """
    d = loadmat(REC / f'{name}.mat', squeeze_me=True, struct_as_record=False)
    m = d['meas']
    m.ref = conj(m.orient)
    m.xhat_matlab = d['xhat'].x
    return m


def calibration():
    """Calibration constants (stationary recording, done in the course lab) from data/calibration.json."""
    c = json.loads((ROOT / 'data' / 'calibration.json').read_text())
    a = {k: np.array(v) for k, v in c.items()}
    return a


def default_config(**kw):
    c = calibration()
    m0 = np.array([0, np.hypot(*c['mag_mean'][:2]), c['mag_mean'][2]])
    return Config(g0=c['acc_mean'], m0=m0, Ra=c['acc_cov'], Rw=c['gyr_cov'], Rm=c['mag_cov'],
                  L0=c['mag_mean'], **kw)
