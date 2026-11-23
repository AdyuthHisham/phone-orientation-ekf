# Conventions

* Quaternion $q=[q_0,q_1,q_2,q_3]$, scalar first, Hamilton product, unit norm, sign fixed by $q_0\ge 0$.
* **Filter convention:** $Q(q)$ (the usual rotation matrix of $q$) maps a **world** vector into the **sensor** frame,
  so $y_\text{acc}=Q(q)g_0$, $y_\text{mag}=Q(q)m_0$.
* World frame: $z$ up, $y$ toward magnetic north, so $g_0=[0,0,g]^\top$ and $m_0=[0,\,m_h,\,m_v]^\top$ with
  $m_h=\sqrt{m_x^2+m_y^2}$ of the calibration mean.
* **Phone convention:** the `orient` quaternion saved by the app satisfies $y_\text{acc}=Q(q_\text{phone})^\top g_0$
  (checked on all recordings with motion: median cosine between predicted and measured gravity direction is 1.000 for $Q^\top$ but only 0.87–0.96 for $Q$).
  The loader therefore uses $q_\text{ref}=\bar q_\text{phone}$ (conjugate). The original `testCode.m`/`Task12.m` compared the two directly.
* Sensor axes: Android convention, $x$ right, $y$ up the screen, $z$ out of the screen.

## Error metrics
For orientations $q_\text{est},q_\text{ref}$ (filter convention) form $e=\bar q_\text{est}\otimes q_\text{ref}$ and split it into swing and twist about the world vertical:

* total $=2\arccos|e_0|$
* tilt (roll/pitch error, needs no magnetometer) $=2\arccos\sqrt{e_0^2+e_3^2}$
* heading (yaw error) $=2\,\mathrm{atan2}(e_3,e_0)$, wrapped to $[-180^\circ,180^\circ]$

RMSE and maxima skip the first 5 s (the filter starts from identity, not from the true attitude) and samples where the reference is NaN.
Per-axis Euler RMSE is not reported: Euler angles are ill-defined near $\pm90^\circ$ pitch, which these recordings reach.
