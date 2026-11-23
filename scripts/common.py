"""Shared helpers for the experiment / figure / animation scripts."""
import sys
from pathlib import Path
import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / 'src' / 'python'))
from orientation_ekf import load, run_filter, default_config  # noqa: E402
from orientation_ekf.metrics import summarize, errors  # noqa: E402

RES = ROOT / 'results'
FIG = RES / 'figures'
ANIM = RES / 'animations'
for d in (FIG, ANIM):
    d.mkdir(parents=True, exist_ok=True)

MAIN = 'A_all_sensors'          # primary recording: 140 s, all three sensors, lots of motion
SECOND = 'B_all_sensors_long'   # second recording: 182 s, mostly at rest after t = 95 s

# Okabe-Ito colour-blind-safe palette
C = dict(ref='#000000', ekf='#0072B2', gyro='#D55E00', acc='#009E73', mag='#CC79A7',
         grey='#7f7f7f', warn='#E69F00', sky='#56B4E9')


def first_ref(m):
    return int(np.where(~np.isnan(m.ref).any(0))[0][0])


def configs(m):
    """Named filter configurations used in the ablation. 'ref' init = start at the phone's first orientation."""
    x0 = m.ref[:, first_ref(m)]
    return {
        'gyro only':                        dict(use_acc=False, use_mag=False, x0=x0),
        'gyro + acc':                       dict(use_mag=False, acc_gate=None, x0=x0),
        'gyro + acc (gated)':               dict(use_mag=False, x0=x0),
        'gyro + mag':                       dict(use_acc=False, mag_thr=None, x0=x0),
        'gyro + mag (gated)':               dict(use_acc=False, x0=x0),
        'gyro + acc + mag':                 dict(acc_gate=None, mag_thr=None, x0=x0),
        'gyro + acc + mag, acc gate':       dict(mag_thr=None, x0=x0),
        'gyro + acc + mag, mag gate':       dict(acc_gate=None, x0=x0),
        'gyro + acc + mag, both gates':     dict(x0=x0),
        'final (identity init)':            dict(),
        'final, course process model':      dict(legacy_process_model=True),
        'final + |m| norm gate (extra)':    dict(mag_norm_tol=10.0),
    }


def run(m, **kw):
    return run_filter(m, default_config(**kw))
