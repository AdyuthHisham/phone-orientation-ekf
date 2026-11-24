# Findings from re-examining the coursework code

Everything below was found by running the code on the recorded data (`scripts/`, `tests/`).

## 1. Port validation
The Python port reproduces the quaternion trajectory stored by the original MATLAB live runs in all six recordings
(max abs difference ≤ 6.5e-8, `results/validation.csv`). The live settings were recovered by searching over candidate settings until the
stored output was reproduced: acc gate 9.6–9.95 m/s², α = 0.6, threshold 0.9, $\hat L_0$ = calibration mean (recordings A, C, E); gates off (B, D, F).
The lab calibration constants come from `calAcc/calGyr/calMag` stored with the recordings.
The MATLAB code in `src/matlab` is a refactor of that code. It was run under **GNU Octave 11.3** (not MATLAB itself): `reproduceMetrics.m` gives the same metrics as the Python port (tilt RMSE 2.05°, heading RMSE 54.72°) and reproduces the stored output with the course-style process model to 8.8e-9. The live script `runEkfLive.m` could not be run (needs the phone, the course jar and `OrientationView`).

## 2. The course-style process model is inconsistent with the measurement model
The template's $S(\omega)$ is the matrix of *right* multiplication, $\dot q=\tfrac12 q\otimes\omega$, which holds for $q$ body→world.
The measurement model $y=Q(q)v_0$ uses $q$ world→body. Starting from the phone's own orientation and integrating the gyro for 2 s, the tilt error
(median / 90th percentile over 23 windows, recording A) is 33° / 167° with the template matrix and 2.5° / 6.1° with $S=-L(\omega)$.
The EKF still tracked tilt because the very small accelerometer covariance makes the update dominate, but with the consistent model:

| recording A, all sensors, gates on | tilt RMSE | tilt max |
|---|---|---|
| template process model (as submitted) | 9.9° | 61.7° |
| consistent process model (this repo) | 2.1° | 6.9° |

(`results/metrics.csv`, rows "final" and "final, course process model".) On recording B the template model is far worse (tilt RMSE 127°).

## 3. Reference convention
The phone's quaternion is the conjugate of the filter convention ([conventions](conventions.md)); the original tuning script `testCode.m`
compared Euler angles of the two directly, so its RMSE-based choice of α and threshold compared mismatched orientations. Its tuned values are not recorded anywhere in the repo.

## 4. Bug in `filterOfflineWithParams.m`
`m0 = [0; sqrt(-28.3899^2 + -15.4134^2) - 15.4134; -63.7006]`: in MATLAB `-28.3899^2` is `-(28.3899^2)`, so the square root is of a negative number and `m0` is **complex**
(the later `real(...)` in `testCode.m` hides the symptom). The live code `Task3.m` computes `m0` correctly from the calibration mean, so the saved live recordings are unaffected. This repo uses the correct form.
Other hard-coded constants there (`g0`, `Ra`, `Rw`, `Rm`, `L_hat_old`) differ from the stored calibration; `Task12.m` loads a non-existent `SensorData_testing_acc_gyro.mat`.

## 5. Calibration constants vs. a stationary segment
Re-estimating from the first 3 s of recording B gives accelerometer variances 4–20× larger than the stored calibration (`results/calibration_check.json`):
noise statistics depend strongly on the recording conditions, one reason the covariance in `P` is not credible (see below).

## 6. Covariance is over-confident
P maps to ~0.01–0.03° attitude uncertainty while the actual tilt error against the phone is 0.5–5° (`results/figures/uncertainty.png`). Only sensor noise is modelled; linear acceleration, bias and magnetic disturbance are not.
