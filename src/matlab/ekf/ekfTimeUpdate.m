function [x, P] = ekfTimeUpdate(x, P, omega, T, Rw, legacy)
% EKFTIMEUPDATE  Gyroscope time update of the quaternion EKF.
%   x <- normalise((I + T/2 S(omega)) x),  P <- F P F' + G Rw G',  G = T/2 S_q(x)
  if nargin < 6, legacy = false; end
  F = eye(4) + 0.5*T*Somega(omega, legacy);
  G = 0.5*T*Sq(x, legacy);
  x = normalizeQuat(F*x);
  P = F*P*F' + G*Rw*G';
end
