function [attack_flag, confidence, detection_time, residuals] = avr_detector(y_meas, t, plant_tf, r_ref, detector_config)
% AVR_DETECTOR Residual-based detector using a Kalman-like observer
% [attack_flag, confidence, detection_time, residuals] = avr_detector(y_meas, t, plant_tf, r_ref, detector_config)
%
% Inputs:
%  y_meas: N×1 measured output (possibly attacked)
%  t: N×1 time vector
%  plant_tf: nominal closed-loop transfer function from reference to output
%            (the same model used to generate the baseline response)
%  r_ref: N×1 reference signal (setpoint)
%  detector_config: struct with fields:
%     .baseline_window (float, seconds) default 5
%     .window_size (int, samples) default 100
%     .Q (matrix) process noise covariance default 1e-6*I
%     .R (matrix) measurement noise covariance default 1e-4
%     .threshold_factor (float) default 2
%
% Outputs:
%  attack_flag: boolean
%  confidence: detection metric value at detection
%  detection_time: time of first detection (NaN if none)
%  residuals: N×1 residual history

if nargin < 5 || isempty(detector_config)
    detector_config = struct();
end
% Tightened defaults to reduce false/early triggers; user can override via detector_config
if ~isfield(detector_config,'baseline_window'), detector_config.baseline_window = 5; end
if ~isfield(detector_config,'window_size'), detector_config.window_size = 100; end
if ~isfield(detector_config,'threshold_factor'), detector_config.threshold_factor = 3; end
if ~isfield(detector_config,'Q'), detector_config.Q = 1e-6; end
if ~isfield(detector_config,'R'), detector_config.R = 1e-4; end
if ~isfield(detector_config,'min_consecutive'), detector_config.min_consecutive = 5; end
if ~isfield(detector_config,'startup_suppress'), detector_config.startup_suppress = detector_config.baseline_window; end
if ~isfield(detector_config,'confidence_cap'), detector_config.confidence_cap = 1e12; end
if ~isfield(detector_config,'residual_clamp'), detector_config.residual_clamp = 1e6; end

% Prepare
t = t(:); y_meas = y_meas(:); r_ref = r_ref(:);
N = length(t);
if length(y_meas) ~= N || length(r_ref) ~= N
    error('t, y_meas, and r_ref must have same length');
end

% Build continuous-state model from the nominal closed-loop reference-to-output model
try
    sys = minreal(plant_tf);
    ss_sys = ss(sys);
catch
    % If plant_tf is already ss
    if isa(plant_tf, 'ss')
        ss_sys = plant_tf;
    else
        error('plant_tf must be a tf or ss model');
    end
end

A = ss_sys.A; B = ss_sys.B; C = ss_sys.C; D = ss_sys.D;

nx = size(A,1);

% For Kalman-like observer, use LQE (continuous)
% Ensure Q and R are appropriate sizes
Q = detector_config.Q * eye(nx);
R = detector_config.R;

% Compute steady-state Kalman gain (continuous LQE)
try
    [~,~,Kf] = lqe(A, eye(nx), C, Q, R);
catch
    % fallback: use place to place eigenvalues
    p = eig(A) * 0.8;
    try
        Kf = place(A', C', p)';
    catch
        Kf = eye(nx, size(C,1));
    end
end

% Initialize
xhat = zeros(nx,1);
residuals = zeros(N,1);
threshold = NaN;
attack_flag = false;
confidence = 0;
detection_time = NaN;
exceed_count = 0;

% Precompute indices for the baseline window. Use a strict boundary so the
% first attack sample at t == baseline_window is not folded into the nominal baseline.
idx_baseline_end = find(t < detector_config.baseline_window, 1, 'last');
if isempty(idx_baseline_end)
    idx_baseline_end = min(N, round(detector_config.baseline_window / (t(2)-t(1))));
end

% Simulate observer with simple Euler integration
for k = 1:N
    if k == 1
        dt = t(1);
    else
        dt = t(k) - t(k-1);
    end

    % predictor
    y_hat = C * xhat + D * r_ref(max(1,k));
    e = y_meas(k) - y_hat;
    % clamp residual to avoid numerical blowups
    rc = detector_config.residual_clamp;
    if ~isfinite(e), e = sign(e)*rc; end
    e = max(min(e, rc), -rc);
    residuals(k) = e;

    % update xhat
    xhat_dot = A * xhat + B * r_ref(max(1,k)) + Kf * e;
    xhat = xhat + xhat_dot * dt;

    % Baseline threshold calculation
    if k == idx_baseline_end
        sigma = std(residuals(1:idx_baseline_end));
        if sigma <= 0
            sigma = 1e-6;
        end
        threshold = detector_config.threshold_factor * sigma;
    end

    % Detection metric using sliding window
    win = detector_config.window_size;
    win_start = max(1, k - win + 1);
        % Use median for robustness to outliers and reduce sensitivity to early transients
        Jk = abs(e) + median(abs(residuals(win_start:k)));

        % Only evaluate detection after baseline threshold computed and startup suppression
        if ~isnan(threshold) && ~attack_flag && t(k) > detector_config.startup_suppress
            if Jk > threshold
                exceed_count = exceed_count + 1;
            else
                exceed_count = 0;
            end
            if exceed_count >= detector_config.min_consecutive
                attack_flag = true;
                % clamp confidence to avoid numerical extremes
                confidence = min(Jk, detector_config.confidence_cap);
                detection_time = t(k);
            end
        end
end

end
