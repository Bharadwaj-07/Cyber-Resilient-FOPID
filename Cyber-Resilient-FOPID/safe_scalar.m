function v = safe_scalar(x, cap)
% SAFE_SCALAR Return finite scalar otherwise NaN
    if ~isfinite(x) || abs(x) > cap
        v = NaN;
        return;
    end
    v = x;
end
