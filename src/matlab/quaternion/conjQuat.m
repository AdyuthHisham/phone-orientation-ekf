function q = conjQuat(q)
% CONJQUAT  Conjugate of quaternions stored as columns (4 x N).
  q(2:4, :) = -q(2:4, :);
end
