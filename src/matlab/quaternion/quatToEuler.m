function eulerDeg = quatToEuler(q)
% QUATTOEULER  [roll; pitch; yaw] in degrees (ZYX, body->world) of filter-convention quaternions (4 x N).
  N = size(q, 2); eulerDeg = zeros(3, N);
  for k = 1:N
    R = Qq(q(:, k))';   % body -> world
    eulerDeg(:, k) = rad2deg([atan2(R(3,2), R(3,3)); asin(max(min(-R(3,1), 1), -1)); atan2(R(2,1), R(1,1))]);
  end
end
