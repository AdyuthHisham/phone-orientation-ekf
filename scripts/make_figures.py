"""All static figures -> results/figures/*.png (+ pipeline.svg). Needs results/*.csv from run_experiments.py."""
import csv, json
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib.patches import FancyBboxPatch, FancyArrowPatch
from common import *
from orientation_ekf.quaternion import to_euler_zyx, Sq

plt.rcParams.update({'figure.facecolor': 'white', 'axes.facecolor': 'white', 'savefig.facecolor': 'white',
                     'axes.grid': True, 'grid.alpha': .25, 'axes.spines.top': False, 'axes.spines.right': False,
                     'font.size': 10, 'axes.titlesize': 11, 'axes.titleweight': 'bold', 'legend.frameon': False,
                     'figure.dpi': 100, 'savefig.dpi': 170, 'lines.linewidth': 1.2})


def save(fig, name):
    fig.savefig(FIG / name, bbox_inches='tight')
    plt.close(fig)
    print('wrote', name)


def read_csv(name):
    rows = list(csv.DictReader(open(RES / name)))
    return [{k: (float(v) if k not in ('recording', 'config') else v) for k, v in r.items()} for r in rows]


def shade(ax, t, mask, color, label=None, alpha=.25):
    """Shade contiguous True runs of mask over time t."""
    idx = np.flatnonzero(np.diff(np.r_[0, mask.astype(int), 0]))
    for i, (a, b) in enumerate(zip(idx[::2], idx[1::2])):
        ax.axvspan(t[a], t[min(b, len(t) - 1)], color=color, alpha=alpha, lw=0, label=label if i == 0 else None)


# ---------------------------------------------------------------- b. sensor streams
def fig_sensor_streams(m, r):
    t, an, mn = m.t, np.linalg.norm(m.acc, axis=0), np.linalg.norm(m.mag, axis=0)
    cal = default_config()
    fig, ax = plt.subplots(5, 1, figsize=(11, 10), sharex=True, height_ratios=[1.2, .8, 1.2, 1.2, .8])
    ax[0].plot(t, m.acc.T, lw=.6); ax[0].set_ylabel('acc [m/s²]'); ax[0].set_title('Accelerometer')
    ax[0].legend(['x', 'y', 'z'], ncol=3, loc='upper right')
    ax[1].plot(t, an, color='k', lw=.6); ax[1].axhspan(*default_config().acc_gate, color=C['acc'], alpha=.35, label='accepted band 9.6–9.95')
    ax[1].set_ylim(0, 25); ax[1].set_ylabel('|acc| [m/s²]'); ax[1].legend(loc='upper right')
    ax[1].annotate('shaking / hand motion:\n|a| ≠ g, so gravity direction is corrupted', xy=(42, 22), xytext=(50, 14), fontsize=9,
                   arrowprops=dict(arrowstyle='->', color=C['grey']))
    ax[2].plot(t, m.gyr.T, lw=.6); ax[2].set_ylabel('gyro [rad/s]'); ax[2].set_title('Gyroscope')
    ax[2].legend(['x', 'y', 'z'], ncol=3, loc='upper right')
    ax[2].annotate('quiet at rest, bursts of fast rotation', xy=(110, 0.1), xytext=(88, -4.7), fontsize=9,
                   arrowprops=dict(arrowstyle='->', color=C['grey']))
    ax[3].plot(t, m.mag.T, lw=.6); ax[3].set_ylabel('mag [µT]'); ax[3].set_title('Magnetometer')
    ax[3].legend(['x', 'y', 'z'], ncol=3, loc='upper right')
    ax[4].plot(t, mn, color='k', lw=.6); ax[4].axhline(np.linalg.norm(cal.m0), color=C['mag'], lw=1.5, label=f'calibrated |m₀| = {np.linalg.norm(cal.m0):.0f} µT')
    ax[4].set_ylabel('|mag| [µT]'); ax[4].set_xlabel('time [s]'); ax[4].legend(loc='upper right')
    ax[4].annotate('indoor disturbances: field strength jumps\nbetween ~70 and ~145 µT (should be constant)', xy=(26, 4), xytext=(40, 20), fontsize=9,
                   arrowprops=dict(arrowstyle='->', color=C['grey']))
    fig.suptitle(f'Raw sensor streams, recording A ({t[-1]:.0f} s, ~125 Hz)', fontweight='bold', y=.995)
    fig.tight_layout()
    save(fig, 'sensor_streams.png')


# ---------------------------------------------------------------- calibration noise
def fig_calibration(m):
    cal = json.load(open(ROOT / 'data' / 'calibration.json'))
    n = int(3.0 / 0.008)
    fig, ax = plt.subplots(1, 3, figsize=(11, 3.2))
    for a, (k, i, u, nm) in zip(ax, [('acc', 2, 'm/s²', 'Accelerometer z'), ('gyr', 0, 'rad/s', 'Gyroscope x'), ('mag', 0, 'µT', 'Magnetometer x')]):
        x = getattr(m, {'acc': 'acc', 'gyr': 'gyr', 'mag': 'mag'}[k])[i, :n]
        a.hist(x, bins=30, density=True, color=C['sky'], alpha=.8, label='data, first 3 s at rest (rec. B)')
        g = np.linspace(x.min() - 3 * x.std(), x.max() + 3 * x.std(), 300)
        mu, var = cal[k + '_mean'][i], cal[k + '_cov'][i][i]
        a.plot(g, np.exp(-(g - mu) ** 2 / 2 / var) / np.sqrt(2 * np.pi * var), color=C['gyro'], label='N(calibration mean, var)')
        a.set_title(nm); a.set_xlabel(u); a.set_yticks([])
    ax[0].legend(fontsize=8, loc='upper left')
    fig.suptitle('Sensor noise model: calibration constants vs. a stationary segment', fontweight='bold', y=1.03)
    fig.tight_layout()
    save(fig, 'calibration_noise.png')


# ---------------------------------------------------------------- c. drift
def fig_gyro_drift():
    fig, ax = plt.subplots(2, 2, figsize=(11, 5.6), sharey='row')
    for j, rec in enumerate((MAIN, SECOND)):
        m = load(rec)
        g = run(m, **configs(m)['gyro only'])
        e = run(m)
        for i, (idx, nm) in enumerate(((1, 'tilt error (roll/pitch) [°]'), (2, 'heading error [°]'))):
            a = ax[i, j]
            a.plot(m.t, errors(m.t, g['x'], m.ref, 0)[idx] if i == 0 else np.abs(errors(m.t, g['x'], m.ref, 0)[idx]), color=C['gyro'], label='gyro integration only')
            a.plot(m.t, errors(m.t, e['x'], m.ref, 0)[idx] if i == 0 else np.abs(errors(m.t, e['x'], m.ref, 0)[idx]), color=C['ekf'], label='EKF (gyro + acc + mag)')
            a.set_ylabel(nm if j == 0 else '')
            if i == 0: a.set_title(f"recording {rec[0]} ({m.t[-1]:.0f} s)")
            if i == 1: a.set_xlabel('time [s]'); 
        ax[1, 0].set_ylabel('|heading error| [°]')
    ax[0, 0].legend(loc='upper left')
    fig.suptitle('Gyro-only integration drifts; fusion keeps the tilt bounded (error vs. the phone\'s own estimate)', fontweight='bold', y=1.0)
    fig.tight_layout()
    save(fig, 'gyro_drift.png')


# ---------------------------------------------------------------- Euler comparison
def fig_euler(m, e, g):
    fig, ax = plt.subplots(4, 1, figsize=(11, 8.5), sharex=True, height_ratios=[1, 1, 1, .9])
    eu = {k: to_euler_zyx(v) for k, v in (('ref', m.ref), ('ekf', e['x']), ('gyro', g['x']))}
    for i, nm in enumerate(('roll', 'pitch', 'yaw')):
        a = ax[i]
        a.plot(m.t, eu['ref'][i], color=C['ref'], ls='--', lw=1, label='phone reference')
        a.plot(m.t, eu['gyro'][i], color=C['gyro'], lw=.9, alpha=.8, label='gyro-only')
        a.plot(m.t, eu['ekf'][i], color=C['ekf'], lw=1.2, label='EKF')
        a.set_ylabel(f'{nm} [°]')
    ax[0].legend(ncol=3, loc='lower right', bbox_to_anchor=(1, 1.0))
    tot, tilt, head = errors(m.t, e['x'], m.ref, 0)
    ax[3].plot(m.t, tilt, color=C['acc'], label='tilt error'); ax[3].plot(m.t, np.abs(head), color=C['mag'], label='|heading error|')
    ax[3].set_ylabel('EKF error [°]'); ax[3].set_xlabel('time [s]'); ax[3].legend(ncol=2, loc='upper right')
    fig.suptitle('Euler angles (ZYX, body→world), recording A', fontweight='bold', x=.01, ha='left', y=.995)
    fig.tight_layout()
    save(fig, 'euler_angles.png')


# ---------------------------------------------------------------- d. ablation
def fig_ablation():
    rows = [r for r in read_csv('metrics.csv') if r['recording'] == MAIN and 'extra' not in r['config'] and 'course' not in r['config']]
    names = [r['config'] for r in rows]
    y = np.arange(len(rows))[::-1]
    fig, ax = plt.subplots(1, 2, figsize=(11, 4.6), sharey=True)
    for a, key, ttl, col in ((ax[0], 'tilt_rmse', 'Tilt error (roll/pitch) RMSE [°]', C['acc']), (ax[1], 'heading_rmse', 'Heading error (yaw) RMSE [°]', C['mag'])):
        a.barh(y, [r[key] for r in rows], color=col)
        for yi, r in zip(y, rows):
            a.text(r[key] + 1, yi, f"{r[key]:.1f}", va='center', fontsize=8)
        a.set_title(ttl); a.set_xlim(0, max(r[key] for r in rows) * 1.15)
    ax[0].set_yticks(y, names)
    fig.suptitle('Ablation on recording A: what each sensor and each gate contributes', fontweight='bold', y=1.02)
    fig.tight_layout()
    save(fig, 'ablation.png')


# ---------------------------------------------------------------- e. outlier gates
def fig_outliers(m, r):
    cfg = default_config()
    t = m.t
    w = (t > 12) & (t < 45)
    an, mn = np.linalg.norm(m.acc, axis=0), np.linalg.norm(m.mag, axis=0)
    innov = np.linalg.norm(m.mag - r['L_hat'], axis=0)
    fig, ax = plt.subplots(4, 1, figsize=(11, 9), sharex=True, height_ratios=[1, 1, 1, 1])
    a = ax[0]
    a.axhspan(*cfg.acc_gate, color=C['acc'], alpha=.3, label='gate: 9.6 < |a| < 9.95')
    ok = r['acc_ok']
    a.plot(t[w], an[w], color=C['grey'], lw=.5)
    a.scatter(t[w & ~ok], an[w & ~ok], s=3, color=C['gyro'], label='rejected', zorder=3)
    a.scatter(t[w & ok], an[w & ok], s=3, color=C['acc'], label='accepted', zorder=3)
    a.set_ylim(5, 16); a.set_ylabel('|acc| [m/s²]'); a.legend(ncol=3, loc='upper right', markerscale=3)
    a.set_title('Accelerometer gate: use gravity only when the phone is not accelerating')
    a = ax[1]
    a.plot(t[w], mn[w], color='k', lw=.8, label='|m| measured'); a.plot(t[w], np.linalg.norm(r['L_hat'][:, w], axis=0), color=C['mag'], lw=1, label='|L̂| running mean')
    a.axhline(np.linalg.norm(cfg.m0), color=C['sky'], ls='--', label='calibrated |m₀|'); a.set_ylabel('[µT]'); a.legend(ncol=3, loc='upper right')
    a.set_title('Magnetic disturbance in this window')
    a = ax[2]
    mo = r['mag_ok']
    a.axhline(cfg.mag_thr, color=C['warn'], label=f'threshold = {cfg.mag_thr}')
    a.scatter(t[w & ~mo], innov[w & ~mo], s=4, color=C['gyro'], label='rejected', zorder=3)
    a.scatter(t[w & mo], innov[w & mo], s=4, color=C['mag'], label='accepted', zorder=3)
    a.set_yscale('log'); a.set_ylabel('‖m − L̂‖ [µT]'); a.legend(ncol=3, loc='upper right', markerscale=3)
    a.set_title(f'Adaptive magnetometer gate (α = {cfg.mag_alpha}): rejects jumps, but accepts the field once L̂ has caught up')
    a = ax[3]
    on = run(m); off = run(m, mag_thr=None, acc_gate=None)
    a.plot(t[w], np.abs(errors(t, off['x'], m.ref, 0)[2])[w], color=C['gyro'], label='no gates')
    a.plot(t[w], np.abs(errors(t, on['x'], m.ref, 0)[2])[w], color=C['ekf'], label='with gates')
    a.set_ylabel('|heading error| [°]'); a.set_xlabel('time [s]'); a.legend(loc='upper right'); a.set_title('Effect on heading')
    fig.tight_layout()
    save(fig, 'outlier_gates.png')


# ---------------------------------------------------------------- f. covariance
def fig_uncertainty(m, r):
    t = m.t
    sig = np.empty((len(t), 3))
    for k in range(len(t)):
        S = Sq(r['x'][:, k])
        sig[k] = np.sqrt(np.diag(4 * S.T @ r['P'][k] @ S))
    sig = np.degrees(sig)
    tilt = errors(t, r['x'], m.ref, 0)[1]
    fig, ax = plt.subplots(2, 1, figsize=(11, 6), sharex=False, height_ratios=[1, 1])
    a = ax[0]
    for i, (nm, c) in enumerate(zip('xyz', (C['gyro'], C['acc'], C['ekf']))):
        a.plot(t, sig[:, i], color=c, lw=.8, label=f'σ about body {nm}')
    a.plot(t, tilt, color=C['grey'], lw=.6, alpha=.7, label='actual tilt error (vs. phone)')
    a.set_yscale('log'); a.set_ylabel('1σ attitude uncertainty [°]'); a.set_xlabel('time [s]'); a.legend(ncol=4, loc='upper right', fontsize=8)
    a.set_title('Filter covariance P over the full run, mapped to rotation angles (log scale)')
    a = ax[1]
    z = (t > 60) & (t < 64)
    for i, (nm, c) in enumerate(zip('xyz', (C['gyro'], C['acc'], C['ekf']))):
        a.plot(t[z], sig[z, i], color=c, lw=1, label=f'σ about body {nm}')
    rej = z & ~r['acc_ok']
    a.scatter(t[rej], np.full(rej.sum(), sig[z].max() * 1.03), marker='|', s=40, color=C['gyro'], label='acc update rejected by gate')
    a.set_xlabel('time [s]'); a.set_ylabel('σ [°]'); a.legend(ncol=4, loc='lower right', fontsize=8)
    a.set_title('Zoom (60–64 s): P grows in the gyro time update and shrinks at accepted measurement updates')
    fig.tight_layout()
    save(fig, 'uncertainty.png')


# ---------------------------------------------------------------- g. sensitivity
def fig_sensitivity():
    rows = read_csv('sensitivity.csv')
    al = sorted({r['alpha'] for r in rows}); th = sorted({r['threshold'] for r in rows})
    fig, ax = plt.subplots(2, 3, figsize=(12, 7))
    for i, rec in enumerate((MAIN, SECOND)):
        for j, (key, ttl, cmap) in enumerate((('tilt_rmse', 'tilt RMSE [°]', 'viridis_r'), ('heading_rmse', 'heading RMSE [°]', 'viridis_r'), ('mag_used', 'fraction of mag samples accepted', 'cividis'))):
            Z = np.array([[next(r[key] for r in rows if r['recording'] == rec and r['alpha'] == a and r['threshold'] == t) for t in th] for a in al])
            a_ = ax[i, j]; im = a_.imshow(Z, cmap=cmap, aspect='auto', origin='lower')
            a_.set_xticks(range(len(th)), [f'{t:g}' for t in th]); a_.set_yticks(range(len(al)), [f'{a:g}' for a in al]); a_.grid(False)
            for (yy, xx), v in np.ndenumerate(Z):
                a_.text(xx, yy, f'{v:.2f}' if key == 'mag_used' else f'{v:.1f}', ha='center', va='center', fontsize=7, color='w' if v < Z.mean() * (1 if key == 'mag_used' else 1.0) and key != 'mag_used' or key == 'mag_used' and v < .6 else 'k')
            a_.add_patch(plt.Rectangle((th.index(0.9) - .5, al.index(0.6) - .5), 1, 1, fill=False, ec=C['warn'], lw=2))
            a_.set_title(f'rec {rec[0]}: {ttl}', fontsize=10); a_.set_xlabel('threshold [µT]'); a_.set_ylabel('α')
            fig.colorbar(im, ax=a_, fraction=.046, pad=.03)
    fig.suptitle('Sensitivity of the magnetometer gate to α and threshold (orange box = values used in the live run)', fontweight='bold', y=1.0)
    fig.tight_layout()
    save(fig, 'sensitivity.png')


# ---------------------------------------------------------------- h. pipeline
def fig_pipeline():
    fig, ax = plt.subplots(figsize=(12, 3.9)); ax.set_xlim(0, 12); ax.set_ylim(-.2, 4.1); ax.axis('off')
    def box(x, y, w, h, txt, fc, ec='#333', fs=9):
        ax.add_patch(FancyBboxPatch((x, y), w, h, boxstyle='round,pad=0.03,rounding_size=0.1', fc=fc, ec=ec, lw=1.2))
        ax.text(x + w / 2, y + h / 2, txt, ha='center', va='center', fontsize=fs)
    def arrow(a, b, **k):
        ax.add_patch(FancyArrowPatch(a, b, arrowstyle='-|>', mutation_scale=12, color='#333', lw=1.2, **k))
    box(.1, 2.7, 1.9, .8, 'Gyroscope ω\n(rad/s)', '#fbe3d3'); box(.1, 1.55, 1.9, .8, 'Accelerometer a\n(m/s²)', '#cfeee3'); box(.1, .4, 1.9, .8, 'Magnetometer m\n(µT)', '#f3dbe8')
    box(2.9, 2.55, 2.2, 1.1, 'Time update\nq ← (I + ½T·S(ω)) q\nP ← FPFᵀ + G·Rω·Gᵀ', '#fbe3d3', fs=8.5)
    box(2.9, 1.4, 2.2, 1.1, 'Gate: 9.6 < |a| < 9.95\nthen EKF update\ny = Q(q)·g₀', '#cfeee3', fs=8.5)
    box(2.9, .25, 2.2, 1.1, 'Gate: ‖m − L̂‖ < thr\nthen EKF update\ny = Q(q)·m₀', '#f3dbe8', fs=8.5)
    box(6.0, 1.4, 1.9, 1.1, 'Normalise\nq ← q/‖q‖\nq₀ ≥ 0', '#dbe9f6')
    box(8.8, 1.4, 1.5, 1.1, 'Quaternion q\n+ covariance P', '#dbe9f6')
    box(10.6, 2.45, 1.3, 1.0, 'Euler angles\n(roll,pitch,yaw)', '#eeeeee', fs=8.5); box(10.6, .45, 1.3, 1.0, '3D viewer\n(phone box)', '#eeeeee', fs=8.5)
    for y in (3.1, 1.95, .8): arrow((2.0, y), (2.9, y if y != 1.95 else 1.95))
    ax.text(2.45, 3.3, '', fontsize=1)
    arrow((5.1, 3.1), (6.0, 2.2)); arrow((5.1, 1.95), (6.0, 1.95)); arrow((5.1, .8), (6.0, 1.7))
    arrow((7.9, 1.95), (8.8, 1.95)); arrow((10.3, 2.2), (10.6, 2.9)); arrow((10.3, 1.7), (10.6, 1.0))
    ax.annotate('', xy=(4.0, 3.68), xytext=(9.5, 2.52), arrowprops=dict(arrowstyle='-|>', connectionstyle='arc3,rad=0.28', color='#888', ls='--'))
    ax.text(6.4, 3.4, 'previous (q, P)', color='#666', fontsize=8.5)
    ax.text(.1, .02, 'Per-sample order (as in the live code): accelerometer update → gyro time update → magnetometer update', fontsize=9, color='#444', va='bottom')
    fig.savefig(FIG / 'pipeline.svg', bbox_inches='tight'); save(fig, 'pipeline.png')


def main():
    m = load(MAIN)
    r = run(m)
    g = run(m, **configs(m)['gyro only'])
    fig_pipeline(); fig_sensor_streams(m, r); fig_calibration(load(SECOND)); fig_gyro_drift(); fig_euler(m, r, g)
    fig_ablation(); fig_outliers(m, r); fig_uncertainty(m, r); fig_sensitivity()


if __name__ == '__main__':
    main()
