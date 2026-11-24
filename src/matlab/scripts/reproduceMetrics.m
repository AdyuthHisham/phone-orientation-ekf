% REPRODUCEMETRICS  Run the EKF on recording A and print the headline metrics (MATLAB twin of src/python/run_demo.py).
here = fileparts(mfilename('fullpath'));
addpath(genpath(fullfile(here, '..')));
d = load(fullfile(here, '..', '..', '..', 'data', 'recordings', 'A_all_sensors.mat'));
meas = d.meas;
out = runEkfOffline(meas, ekfConfig());

ref = conjQuat(meas.orient);   % phone's quaternion uses the opposite convention (docs/conventions.md)
ok = ~any(isnan(ref), 1) & meas.t >= 5;   % skip the 5 s burn-in
[tot, tilt, head] = quatError(out.x(:, ok), ref(:, ok));
fprintf('total RMSE %.2f deg | tilt RMSE %.2f deg | heading RMSE %.2f deg\n', ...
        sqrt(mean(tot.^2)), sqrt(mean(tilt.^2)), sqrt(mean(head.^2)));

% sanity check against the stored course-run output
stored = runEkfOffline(meas, ekfConfig());
fprintf('max |x - stored xhat.x|: %.2e\n', max(abs(stored.x(:) - d.xhat.x(:))));
