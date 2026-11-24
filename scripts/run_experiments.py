"""Run every numerical experiment and write results/*.csv|json. ~1-2 min on 8 cores."""
import csv, json
from concurrent.futures import ProcessPoolExecutor
import numpy as np
from common import *

ALPHAS = [0.05, 0.1, 0.2, 0.3, 0.4, 0.6, 0.8, 0.95]
THRS = [0.1, 0.2, 0.3, 0.5, 0.9, 1.5, 3.0, 6.0]
VALIDATION = {  # recording -> filter settings the live MATLAB run used (recovered by search, see docs/validation.md)
    'A_all_sensors': dict(),
    'B_all_sensors_long': dict(acc_gate=None, mag_thr=None),
    'C_gyro_acc_gated': dict(use_mag=False),
    'D_gyro_acc_ungated': dict(use_mag=False, acc_gate=None),
    'E_gyro_mag_gated': dict(use_acc=False),
    'F_gyro_mag_ungated': dict(use_acc=False, mag_thr=None),
}


def _ablation(args):
    rec, name = args
    m = load(rec)
    r = run(m, **configs(m)[name])
    s = summarize(m.t, r['x'], m.ref)
    s.update(recording=rec, config=name, acc_used=float(r['acc_ok'].mean()), mag_used=float(r['mag_ok'].mean()))
    return s


def _sens(args):
    rec, a, th = args
    m = load(rec)
    r = run(m, mag_alpha=a, mag_thr=th)
    s = summarize(m.t, r['x'], m.ref)
    return dict(recording=rec, alpha=a, threshold=th, mag_used=float(r['mag_ok'].mean()), **s)


def write_csv(path, rows):
    with open(path, 'w', newline='') as f:
        w = csv.DictWriter(f, fieldnames=list(rows[0]))
        w.writeheader()
        w.writerows({k: (float(f'{v:.4g}') if isinstance(v, float) else v) for k, v in r.items()} for r in rows)


def main():
    with ProcessPoolExecutor() as ex:
        # 1. ablation
        jobs = [(rec, n) for rec in (MAIN, SECOND) for n in configs(load(rec))]
        abl = list(ex.map(_ablation, jobs))
        write_csv(RES / 'metrics.csv', abl)
        # 2. alpha x threshold sensitivity
        sens = list(ex.map(_sens, [(rec, a, t) for rec in (MAIN, SECOND) for a in ALPHAS for t in THRS]))
        write_csv(RES / 'sensitivity.csv', sens)

    # 3. port vs recorded MATLAB output
    val = []
    for rec, kw in VALIDATION.items():
        m = load(rec)
        r = run(m, **kw)
        val.append(dict(recording=rec, samples=len(m.t), max_abs_diff_quaternion=float(np.abs(r['x'] - m.xhat_matlab).max())))
    write_csv(RES / 'validation.csv', val)

    # 4. gyro-only drift numbers
    drift = {}
    for rec in (MAIN, SECOND):
        m = load(rec)
        r = run(m, **configs(m)['gyro only'])
        _, tilt, head = errors(m.t, r['x'], m.ref, burn_in=0)
        drift[rec] = {f'tilt_error_deg_at_{s}s': round(float(tilt[np.searchsorted(m.t, s)]), 1) for s in (30, 60, 120)}
        drift[rec]['duration_s'] = round(float(m.t[-1]), 1)
    json.dump(drift, open(RES / 'gyro_drift.json', 'w'), indent=1)

    # 5. stationary-segment calibration check (rec B starts at rest)
    m = load(SECOND)
    cal = json.load(open(ROOT / 'data' / 'calibration.json'))
    seg = slice(0, int(3.0 / 0.008))
    chk = {}
    for k, a in (('acc', m.acc), ('gyr', m.gyr), ('mag', m.mag)):
        x = a[:, seg]
        chk[k] = dict(mean_here=x.mean(1).round(4).tolist(), mean_calibration=np.round(cal[k + '_mean'], 4).tolist(),
                      var_diag_here=np.diag(np.cov(x)).round(6).tolist(), var_diag_calibration=np.diag(cal[k + '_cov']).round(6).tolist())
    json.dump(chk, open(RES / 'calibration_check.json', 'w'), indent=1)
    print('done')


if __name__ == '__main__':
    main()
