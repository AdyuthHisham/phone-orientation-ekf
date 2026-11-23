import numpy as np
from .quaternion import error_decomposition

BURN_IN_S = 5.0   # ignore the first seconds: the filter starts from identity, not from the true attitude


def errors(t, x, ref, burn_in=BURN_IN_S):
    """Per-sample (total, tilt, heading) error in degrees vs the reference; NaN where ref missing / burn-in."""
    tot, tilt, head = error_decomposition(x, ref)
    bad = np.isnan(ref).any(0) | (t < burn_in)
    for a in (tot, tilt, head):
        a[bad] = np.nan
    return tot, tilt, head


def summarize(t, x, ref, burn_in=BURN_IN_S):
    tot, tilt, head = errors(t, x, ref, burn_in)
    rmse = lambda a: float(np.sqrt(np.nanmean(a**2)))
    return dict(total_rmse=rmse(tot), total_p95=float(np.nanpercentile(tot, 95)), total_max=float(np.nanmax(tot)),
                tilt_rmse=rmse(tilt), tilt_median=float(np.nanmedian(tilt)), tilt_max=float(np.nanmax(tilt)),
                heading_rmse=rmse(head), heading_median_abs=float(np.nanmedian(np.abs(head))),
                heading_max=float(np.nanmax(np.abs(head))),
                heading_within_20deg=float(np.nanmean(np.abs(head[~np.isnan(head)]) < 20)))
