function q = normalizeQuat(q)
% NORMALIZEQUAT  Renormalise to unit length and pick the sign with q0 >= 0.
  q = q / norm(q);
  if q(1) < 0, q = -q; end
end
