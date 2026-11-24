function [s, accOk, magOk] = ekfStep(s, cfg, t, acc, gyr, mag)
% EKFSTEP  One filter iteration, shared by the offline and live runners.
%   s: struct with x, P, L (running mag mean), tPrev. Order: acc update, gyro time update, mag update.
%   Missing measurements are NaN.
  T = t - s.tPrev; s.tPrev = t;
  accOk = false; magOk = false;

  if cfg.useAcc && ~any(isnan(acc))
    if isempty(cfg.accGate) || (norm(acc) > cfg.accGate(1) && norm(acc) < cfg.accGate(2))
      [s.x, s.P] = ekfMeasUpdate(s.x, s.P, acc, cfg.Ra, cfg.g0);
      accOk = true;
    end
  end

  if cfg.useGyr && ~any(isnan(gyr))
    [s.x, s.P] = ekfTimeUpdate(s.x, s.P, gyr, T, cfg.Rw);
  end

  if cfg.useMag && ~any(isnan(mag))
    magOk = true;
    if ~isempty(cfg.magThreshold)
      s.L = (1 - cfg.magAlpha)*s.L + cfg.magAlpha*mag;   % updated for every sample, also rejected ones
      magOk = norm(mag - s.L) < cfg.magThreshold;
    end
    if magOk
      [s.x, s.P] = ekfMeasUpdate(s.x, s.P, mag, cfg.Rm, cfg.m0);
    end
  end
end
