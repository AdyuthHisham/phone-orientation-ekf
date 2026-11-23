"""3-panel 3D orientation animation -> results/animations/orientation_comparison.gif

Panels: phone's own estimate (reference) | my EKF | gyro integration only (started from the
reference at t=0, never corrected). Window = the 20 s span of recording A with the smallest EKF
heading error that still contains real motion (picked automatically, disclosed in the README).
"""
import io
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d.art3d import Poly3DCollection
from PIL import Image
from common import *
from orientation_ekf.quaternion import Qq

WIN, STEP = 20.0, 10   # window length [s]; one frame per 10 samples (~12.5 fps real time)
DIM = np.array([.075, .155, .02]) * 6   # phone box (x right, y up, z out of screen), enlarged
V = np.array([[sx, sy, sz] for sx in (-1, 1) for sy in (-1, 1) for sz in (-1, 1)]) * DIM / 2
FACES = [[0, 1, 3, 2], [4, 5, 7, 6], [0, 1, 5, 4], [2, 3, 7, 6], [0, 2, 6, 4], [1, 3, 7, 5]]  # last-axis order sx,sy,sz
FACE_COL = ['#9aa5b1', '#dfe6ee', '#b9c3cf', '#b9c3cf', '#b9c3cf', '#b9c3cf']   # face 1 (z=+1) = screen
AXES = [(np.array([1, 0, 0]), C['gyro'], 'x'), (np.array([0, 1, 0]), C['acc'], 'y'), (np.array([0, 0, 1]), C['ekf'], 'z')]


def pick_window(m, e):
    _, tilt, head = errors(m.t, e['x'], m.ref)
    n = int(WIN / .008)
    w = np.linalg.norm(m.gyr, axis=0)
    best = None
    for s in range(0, len(m.t) - n, 125):
        if np.nanmean(w[s:s + n]) < .5:
            continue
        sc = np.nanmean(np.abs(head[s:s + n]))
        if best is None or sc < best[0]:
            best = (sc, s)
    return best[1], best[1] + n


def setup_panel(ax, title, color):
    ax.set_xlim(-.4, .4); ax.set_ylim(-.4, .4); ax.set_zlim(-.4, .4); ax.set_box_aspect((1, 1, 1))
    ax.view_init(18, -55); ax.set_axis_off(); ax.set_title(title, color=color, fontsize=11, fontweight='bold', y=.93)
    ax.plot([-.38, .38], [0, 0], [-.38, -.38], color='#ccc', lw=.8); ax.plot([0, 0], [-.38, .38], [-.38, -.38], color='#ccc', lw=.8)
    ax.text(0, .42, -.38, 'N', color='#888', fontsize=8, ha='center')   # world +y = magnetic north in the filter model
    poly = Poly3DCollection([[V[i] for i in f] for f in FACES], facecolors=FACE_COL, edgecolors='#333', linewidths=.8)
    ax.add_collection3d(poly)
    lines = [ax.plot([0, 0], [0, 0], [0, 0], color=c, lw=2.2)[0] for _, c, _ in AXES]
    return poly, lines


def pose(panel, q):
    poly, lines = panel
    Rbw = Qq(q).T   # body -> world
    poly.set_verts([[Rbw @ V[i] for i in f] for f in FACES])
    for ln, (a, _, _) in zip(lines, AXES):
        p = Rbw @ (a * .3)
        ln.set_data_3d([0, p[0]], [0, p[1]], [0, p[2]])


def main():
    m = load(MAIN)
    e = run(m)
    g = run(m, **configs(m)['gyro only'])
    a, b = pick_window(m, e)
    idx = np.arange(a, b, STEP)
    t = m.t
    _, tilt_e, head_e = errors(t, e['x'], m.ref, 0)
    _, tilt_g, _ = errors(t, g['x'], m.ref, 0)
    print(f'window {t[a]:.1f}-{t[b-1]:.1f} s, {len(idx)} frames')

    fig = plt.figure(figsize=(10, 4.9), dpi=80)
    gs = fig.add_gridspec(2, 3, height_ratios=[3.2, 1], hspace=.05, wspace=0)
    P = [setup_panel(fig.add_subplot(gs[0, i], projection='3d'), tt, c) for i, (tt, c) in
         enumerate((('Phone estimate (reference)', C['ref']), ('EKF (this repo)', C['ekf']), ('Gyro integration only', C['gyro'])))]
    ax = fig.add_subplot(gs[1, :])
    ax.plot(t, tilt_e, color=C['ekf'], lw=1, label='EKF tilt error'); ax.plot(t, tilt_g, color=C['gyro'], lw=1, label='gyro-only tilt error')
    ax.plot(t, np.abs(head_e), color=C['mag'], lw=.8, alpha=.8, label='EKF |heading error|')
    ax.set_xlim(t[a], t[b - 1]); ax.set_ylim(0, 90); ax.set_ylabel('error [°]'); ax.set_xlabel('time [s]')
    ax.legend(ncol=3, loc='upper left', fontsize=8, frameon=False); ax.grid(alpha=.25)
    for s in ('top', 'right'): ax.spines[s].set_visible(False)
    cur = ax.axvline(t[a], color='k', lw=1)
    fig.subplots_adjust(left=.06, right=.99, top=.98, bottom=.14)
    txt = fig.text(.995, .955, '', ha='right', fontsize=8, color='#555')

    frames = []
    for k in idx:
        pose(P[0], m.ref[:, k]); pose(P[1], e['x'][:, k]); pose(P[2], g['x'][:, k])
        cur.set_xdata([t[k], t[k]]); txt.set_text(f't = {t[k]:.1f} s')
        buf = io.BytesIO(); fig.savefig(buf, format='png', facecolor='white'); buf.seek(0)
        frames.append(Image.open(buf).convert('RGB').quantize(64, dither=Image.Dither.NONE))
    frames[0].save(ANIM / 'orientation_comparison.gif', save_all=True, append_images=frames[1:], duration=80, loop=0, optimize=True)
    print('wrote gif', (ANIM / 'orientation_comparison.gif').stat().st_size // 1024, 'kB')


if __name__ == '__main__':
    main()
