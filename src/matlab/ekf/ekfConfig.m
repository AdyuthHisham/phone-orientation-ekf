function cfg = ekfConfig(varargin)
% EKFCONFIG  Filter settings. Calibration constants come from data/calibration.json.
%   cfg = ekfConfig('accGate', [], 'magThreshold', [], 'useMag', false, ...)  overrides fields;
%   an empty gate switches that gate off.
  here = fileparts(mfilename('fullpath'));
  c = jsondecode(fileread(fullfile(here, '..', '..', '..', 'data', 'calibration.json')));
  cfg.g0 = c.acc_mean(:);   cfg.Ra = c.acc_cov;
  cfg.Rw = c.gyr_cov;       cfg.Rm = c.mag_cov;
  cfg.m0 = [0; hypot(c.mag_mean(1), c.mag_mean(2)); c.mag_mean(3)];   % [0; horizontal; vertical] field
  cfg.L0 = c.mag_mean(:);   % initial running mean of the magnetometer
  cfg.accGate = [9.6 9.95];  % accept acc update if |a| inside [m/s^2]
  cfg.magAlpha = 0.6;        % AR(1) coefficient of the running mean
  cfg.magThreshold = 0.9;    % accept mag update if |m - Lhat| < threshold [uT]
  cfg.useAcc = true; cfg.useGyr = true; cfg.useMag = true;
  cfg.x0 = [1; 0; 0; 0];
  for i = 1:2:numel(varargin), cfg.(varargin{i}) = varargin{i+1}; end
end
