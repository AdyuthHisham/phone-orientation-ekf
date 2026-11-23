# Data

All recordings were captured by the project group with a smartphone running the course's Sensor Fusion app
(streamed over TCP into MATLAB, ~125 Hz). They contain only accelerometer, gyroscope, magnetometer, the phone's own
orientation quaternion and the filter output from the original live run. No location, audio or personal data.

| File | Duration | Sensors used in the original live run | Gates in the live run | Used for |
|---|---|---|---|---|
| `A_all_sensors.mat` | 140 s | acc, gyro, mag | acc gate 9.6–9.95, mag gate α=0.6 / thr 0.9 | **main recording**, all figures |
| `B_all_sensors_long.mat` | 182 s | acc, gyro, mag | none | second recording (mostly at rest after 95 s) |
| `C_gyro_acc_gated.mat` | 89 s | gyro, acc | acc gate | port validation only |
| `D_gyro_acc_ungated.mat` | 83 s | gyro, acc | none | port validation only (also holds the calibration variables) |
| `E_gyro_mag_gated.mat` | 145 s | gyro, mag | mag gate | port validation only |
| `F_gyro_mag_ungated.mat` | 112 s | gyro, mag | none | port validation only |

Variables in each file: `meas` (`t` s, `acc` m/s², `gyr` rad/s, `mag` µT, `orient` the phone's quaternion; NaN = not available / sensor not used)
and `xhat` (`t`, `x` quaternion, `P` covariance of the original MATLAB run). Files are unmodified originals.

`orient` is the phone's own fused estimate. It is a **reference, not ground truth**: it also relies on the magnetometer
and is itself disturbed indoors. It uses the opposite quaternion convention to the filter (`docs/conventions.md`).

`calibration.json` holds the mean and covariance of each sensor from a stationary calibration recording made
in the lab (`calAcc`, `calGyr`, `calMag` saved in `D_*.mat` / `F_*.mat`). The raw stationary recording itself is not in the repo.

Not included: the unrelated 120 s run `sensorData.mat` of the original repo (no reference orientation, no
extra information) and the Java library used to receive the stream (course material).
