function val = safe_itae(y, t, cap)
% SAFE_ITAE Bounded ITAE to avoid exploding values from unstable trajectories.
%   val = safe_itae(y, t, cap)
    if any(~isfinite(y)) || max(abs(y)) > cap
        val = NaN;
        return;
    end
    e = abs(1 - y(:));
    val = trapz(t(:), t(:) .* e);
    if ~isfinite(val)
        val = NaN;
    end
end
