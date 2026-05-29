function y = sanitize_signal(y)
% SANITIZE_SIGNAL Replace non-finite samples with the previous valid value.
%   y = sanitize_signal(y)
    y = y(:);
    y(~isfinite(y)) = NaN;
    if all(isnan(y))
        y = zeros(size(y));
        return;
    end

    firstValid = find(~isnan(y), 1, 'first');
    if firstValid > 1
        y(1:firstValid-1) = y(firstValid);
    end

    for k = 2:numel(y)
        if isnan(y(k))
            y(k) = y(k-1);
        end
    end
end
