function S = Somega(w)
% SOMEGA  The matrix S(omega) of the course template: dq/dt = 0.5*S(omega)*q.
  x = w(1); y = w(2); z = w(3);
  S = [0 -x -y -z;  x 0  z -y;  y -z 0  x;  z  y -x 0];
end
