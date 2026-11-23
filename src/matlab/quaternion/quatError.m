function [total, tilt, heading] = quatError(qEst, qRef)
% QUATERROR  Error between two orientations (4 x N, filter convention) in degrees.
%   total   geodesic angle,  tilt  angle between the 'up' directions (roll/pitch error),
%   heading signed rotation about vertical, wrapped to [-180, 180]  (swing-twist split).
  N = size(qEst, 2); e = zeros(4, N);
  for k = 1:N
    a = conjQuat(qEst(:, k)); b = qRef(:, k);
    e(:, k) = [a(1)*b(1)-a(2)*b(2)-a(3)*b(3)-a(4)*b(4);
               a(1)*b(2)+a(2)*b(1)+a(3)*b(4)-a(4)*b(3);
               a(1)*b(3)-a(2)*b(4)+a(3)*b(1)+a(4)*b(2);
               a(1)*b(4)+a(2)*b(3)-a(3)*b(2)+a(4)*b(1)];
  end
  total   = rad2deg(2*acos(min(abs(e(1,:)), 1)));
  tilt    = rad2deg(2*acos(min(hypot(e(1,:), e(4,:)), 1)));
  heading = rad2deg(2*atan2(e(4,:), e(1,:)));
  heading = mod(heading + 180, 360) - 180;
end
