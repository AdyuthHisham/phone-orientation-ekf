function [x, P] = ekfMeasUpdate(x, P, y, R, v0)
% EKFMEASUPDATE  EKF update for a sensor measuring y = Qq(q)*v0 + noise
%   (accelerometer: v0 = g0 gravity, magnetometer: v0 = m0 field, both in the world frame).
  [Q0, Q1, Q2, Q3] = dQqdq(x);
  H = [Q0*v0, Q1*v0, Q2*v0, Q3*v0];          % 3x4 Jacobian
  S = H*P*H' + R;                            % innovation covariance
  K = P*H'/S;                                % Kalman gain
  x = normalizeQuat(x + K*(y - Qq(x)*v0));
  P = P - K*S*K';
end
