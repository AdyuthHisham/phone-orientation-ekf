function S = Somega(w, legacy)
% SOMEGA  Matrix with dq/dt = 0.5*S*q for q = world->body (the convention of Qq).
%   This is -L(w), L = left multiplication by the pure quaternion [0 w].
%   legacy = true gives the course-template matrix (right multiplication), which is only
%   consistent with q = body->world; kept to reproduce the original submission (docs/findings.md).
  if nargin < 2, legacy = false; end
  x = w(1); y = w(2); z = w(3);
  if legacy
    S = [0 -x -y -z;  x 0  z -y;  y -z 0  x;  z  y -x 0];
  else
    S = [0  x  y  z; -x 0  z -y; -y -z 0  x; -z  y -x 0];
  end
end
