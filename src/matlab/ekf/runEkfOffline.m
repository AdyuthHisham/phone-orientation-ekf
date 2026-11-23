function out = runEkfOffline(meas, cfg)
% RUNEKFOFFLINE  Run the EKF over a recorded dataset.
%   meas: struct with t (1xN), acc, gyr, mag (3xN); cfg from ekfConfig.
%   out: x (4xN), P (4x4xN), accOk, magOk (1xN logical).
  if nargin < 2, cfg = ekfConfig(); end
  N = numel(meas.t);
  s = struct('x', cfg.x0, 'P', eye(4), 'L', cfg.L0, 'tPrev', meas.t(1));
  out = struct('x', zeros(4, N), 'P', zeros(4, 4, N), 'accOk', false(1, N), 'magOk', false(1, N));
  for k = 1:N
    [s, out.accOk(k), out.magOk(k)] = ekfStep(s, cfg, meas.t(k), meas.acc(:, k), meas.gyr(:, k), meas.mag(:, k));
    out.x(:, k) = s.x; out.P(:, :, k) = s.P;
  end
end
