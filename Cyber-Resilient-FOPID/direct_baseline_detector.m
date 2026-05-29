function [attack_flag, confidence, detection_time, residuals] = direct_baseline_detector(y_meas, y_nominal, t, detector_cfg)
% Direct residual detector for Phase 5 validation runs.
if nargin < 4 || isempty(detector_cfg)
    detector_cfg = struct();
end
if ~isfield(detector_cfg, 'baseline_window'), detector_cfg.baseline_window = 5; end
if ~isfield(detector_cfg, 'window_size'), detector_cfg.window_size = 40; end
if ~isfield(detector_cfg, 'threshold_factor'), detector_cfg.threshold_factor = 2; end
if ~isfield(detector_cfg, 'min_consecutive'), detector_cfg.min_consecutive = 1; end
if ~isfield(detector_cfg, 'startup_suppress'), detector_cfg.startup_suppress = max(0, detector_cfg.baseline_window - 0.5); end
if ~isfield(detector_cfg, 'confidence_cap'), detector_cfg.confidence_cap = 10; end

y_meas = y_meas(:);
y_nominal = y_nominal(:);
t = t(:);

residuals = y_meas - y_nominal;
residuals = max(min(residuals, 1e6), -1e6);

idx_baseline_end = find(t < detector_cfg.baseline_window, 1, 'last');
if isempty(idx_baseline_end)
    idx_baseline_end = min(length(t), round(detector_cfg.baseline_window / max(eps, t(2)-t(1))));
end

sigma = std(residuals(1:idx_baseline_end));
if ~isfinite(sigma) || sigma <= 0
    sigma = 1e-6;
end
threshold = detector_cfg.threshold_factor * sigma;

attack_flag = false;
confidence = 0;
detection_time = NaN;
exceed_count = 0;

for k = 1:length(t)
    win_start = max(1, k - detector_cfg.window_size + 1);
    window_abs = abs(residuals(win_start:k));
    Jk = mean(window_abs) + std(window_abs) + abs(residuals(k));
    if t(k) > detector_cfg.startup_suppress
        if Jk > threshold
            exceed_count = exceed_count + 1;
        else
            exceed_count = 0;
        end
        if exceed_count >= detector_cfg.min_consecutive
            attack_flag = true;
            confidence = min(Jk, detector_cfg.confidence_cap);
            detection_time = t(k);
            break;
        end
    end
end
end
