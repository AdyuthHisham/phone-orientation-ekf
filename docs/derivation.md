# EKF derivation

State $x=q\in\mathbb R^4$ (world→sensor, [conventions](conventions.md)), covariance $P\in\mathbb R^{4\times4}$.

## Time update (gyroscope)
Kinematics with body angular rate $\omega$ (course template):
$$\dot q=\tfrac12 S(\omega)\,q,\qquad S(\omega)=\begin{pmatrix}0&-\omega_x&-\omega_y&-\omega_z\\\omega_x&0&\omega_z&-\omega_y\\\omega_y&-\omega_z&0&\omega_x\\\omega_z&\omega_y&-\omega_x&0\end{pmatrix}$$
i.e. $\dot q=\tfrac12 q\otimes[0,\omega]$. Euler discretisation, sample time $T$:
$$q_{k+1}=\Big(I+\tfrac T2 S(\omega_k)\Big)q_k=F_kq_k,\qquad P_{k+1}=F_kP_kF_k^\top+G_kR_\omega G_k^\top,\quad G_k=\tfrac T2 S_q(q_k)$$
where $\omega_k=\omega_\text{meas}+e_\omega$, $e_\omega\sim\mathcal N(0,R_\omega)$ and $S_q(q)\,\omega=q\otimes\omega$. Because $F$ is linear in $q$, no linearisation is needed here. After the update $q\leftarrow q/\|q\|$.
The gyroscope noise covariance measured at rest is used as $R_\omega$; no bias state, no model error term.

## Measurement updates (accelerometer, magnetometer)
$$y=Q(q)\,v_0+e,\quad e\sim\mathcal N(0,R)$$
($v_0=g_0$, $R=R_a$ for the accelerometer; $v_0=m_0$, $R=R_m$ for the magnetometer). The model is non-linear in $q$, so linearise:
$$H=\Big[\tfrac{\partial Q}{\partial q_0}v_0,\ \tfrac{\partial Q}{\partial q_1}v_0,\ \tfrac{\partial Q}{\partial q_2}v_0,\ \tfrac{\partial Q}{\partial q_3}v_0\Big]\in\mathbb R^{3\times4}$$
(`dQqdq`, finite-difference-checked in `tests/test_filter.py`) and apply the standard EKF update
$$S=HPH^\top+R,\quad K=PH^\top S^{-1},\quad q\leftarrow q+K\big(y-Q(q)v_0\big),\quad P\leftarrow P-KSK^\top .$$

## Normalisation
Unit norm is enforced by **renormalising after every update** ($q\leftarrow q/\|q\|$, sign fixed so $q_0\ge0$). `P` is not modified by this step (this is renormalisation, not a covariance-consistent pseudo-measurement), so $P$ keeps variance along the radial direction of $q$; `results/figures/uncertainty.png` therefore maps $P$ to rotation angles through the tangent space ($\delta\theta=2S_q(q)^\top\delta q$ in the sensor frame) before plotting.

## Outlier rejection
* **Accelerometer:** update only if $9.6<\|a\|<9.95\ \mathrm{m/s^2}$ (gravity ≈ 9.84 measured at rest): when the phone accelerates, $a\ne g$ and the gravity-direction measurement is wrong.
* **Magnetometer:** running mean $\hat L_k=(1-\alpha)\hat L_{k-1}+\alpha\,m_k$ (updated every sample); update only if $\|m_k-\hat L_k\|<\tau$. Since $m_k-\hat L_k=(1-\alpha)(m_k-\hat L_{k-1})$ the gate rejects *sudden changes* of the field. It cannot reject a disturbance that persists: once $\hat L$ has caught up the field is accepted again (see `results/figures/outlier_gates.png`).
* Live values: $\alpha=0.6$, $\tau=0.9\,\mu$T, $\hat L_0$ = calibration mean.

## Order of operations
The implementation (course template) processes each sample as accelerometer update → gyro time update → magnetometer update, i.e. the accelerometer is applied to the state *before* propagating over the last sample interval (8 ms). It is not the textbook order but the effect is negligible at 125 Hz.
