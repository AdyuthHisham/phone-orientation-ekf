function S = Sq(q)
% SQ  The matrix S(q) of the course template: the gyro noise enters as dq = 0.5*T*S(q)*e.
  q0 = q(1); q1 = q(2); q2 = q(3); q3 = q(4);
  S = [-q1 -q2 -q3;  q0 -q3  q2;  q3  q0 -q1; -q2  q1  q0];
end
