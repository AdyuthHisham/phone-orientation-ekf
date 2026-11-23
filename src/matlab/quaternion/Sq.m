function S = Sq(q, legacy)
% SQ  Noise input matrix matching Somega: the gyro noise enters as dq = 0.5*T*S*e.
  if nargin < 2, legacy = false; end
  q0 = q(1); q1 = q(2); q2 = q(3); q3 = q(4);
  if legacy
    S = [-q1 -q2 -q3;  q0 -q3  q2;  q3  q0 -q1; -q2  q1  q0];
  else
    S = [-q1 -q2 -q3;  q0  q3 -q2; -q3  q0  q1;  q2 -q1  q0];
  end
end
