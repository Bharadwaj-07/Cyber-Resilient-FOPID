function [u, mode_history, switch_times] = avr_switcher(y_meas, t, r_ref, C_2dof_y, C_pid, switcher_config)
% AVR_SWITCHER Hysteresis-based switching between 2DoF and PID controllers
% [u, mode_history, switch_times] = avr_switcher(y_meas, t, r_ref, C_2dof_y, C_pid, switcher_config)
%
% Inputs:
%  y_meas: N×1 measurement (possibly attacked)
%  t: N×1 time vector
%  r_ref: N×1 reference
%  C_2dof_y: controller TF used on error path (C_y)
%  C_pid: PID TF
%  switcher_config: struct with fields:
%    .hysteresis_time (default 2.0)
%    .recovery_time (default 0.5)
%    .sampling_interval (default inferred from t)
%    .initial_mode (1=normal)
%
% Outputs:
%  u: N×1 control signal (controller output)
%  mode_history: N×1 integer mode (1=normal,2=attacking,3=recovery)
%  switch_times: Mx3 [time, from_mode, to_mode]

if nargin < 6 || isempty(switcher_config), switcher_config = struct(); end
if ~isfield(switcher_config,'hysteresis_time'), switcher_config.hysteresis_time = 2.0; end
if ~isfield(switcher_config,'recovery_time'), switcher_config.recovery_time = 0.5; end
if ~isfield(switcher_config,'initial_mode'), switcher_config.initial_mode = 1; end

t = t(:); y_meas = y_meas(:); r_ref = r_ref(:);
N = length(t);
if length(y_meas) ~= N || length(r_ref) ~= N
    error('t, y_meas, and r_ref must have same length');
end

% Compute error signal
e = r_ref - y_meas; % error used by controllers

% Pre-allocate outputs
u = zeros(N,1);
mode_history = zeros(N,1);
switch_times = [];

% Modes: 1=normal (2DoF), 2=attacking (PID), 3=recovery
mode = switcher_config.initial_mode;
mode_history(1) = mode;
last_switch_time = t(1);
recovery_counter = 0;

% We'll implement a simple logic: detect attack from residuals in y_meas by local variance
% Note: The detector should call avr_detector before and pass flags; but to keep switcher modular
% we implement a local quick-check: if recent residual-like fluctuation is high, go to PID.

% We assume the upstream call provided y_meas that already triggers detection; thus, here
% we simply simulate switching based on mode transitions detected by a simple heuristic

% Compute a quick metric (sliding std of y_meas derivative)
dy = [0; diff(y_meas)];
win = max(1, round(0.05 / (t(2)-t(1)))); % 50 ms window
metric = movstd(dy, win);

% Thresholds (tunable)
metric_thresh = 5 * std(metric(1:max(1,round(0.5/(t(2)-t(1))))));

for k = 1:N
    % Mode transitions
    if mode == 1
        if metric(k) > metric_thresh
            from_mode = mode; mode = 2; switch_times(end+1,:) = [t(k), from_mode, mode]; last_switch_time = t(k);
        end
    elseif mode == 2
        % stay in PID for at least recovery_time seconds
        if t(k) - last_switch_time >= switcher_config.recovery_time
            % check if metric has settled
            recent_metric = mean(metric(max(1,k-win):k));
            if recent_metric < metric_thresh/2
                from_mode = mode; mode = 3; switch_times(end+1,:) = [t(k), from_mode, mode]; last_switch_time = t(k);
            end
        end
    elseif mode == 3
        % recovery: require hysteresis_time of quiet before returning to normal
        if t(k) - last_switch_time >= switcher_config.hysteresis_time
            recent_metric = mean(metric(max(1,k-win):k));
            if recent_metric < metric_thresh/2
                from_mode = mode; mode = 1; switch_times(end+1,:) = [t(k), from_mode, mode]; last_switch_time = t(k);
            elseif recent_metric > metric_thresh
                from_mode = mode; mode = 2; switch_times(end+1,:) = [t(k), from_mode, mode]; last_switch_time = t(k);
            end
        end
    end
    mode_history(k) = mode;
end

% Now compute controller outputs by simulating controller TFs on error but applying mode mask
% For each contiguous segment with same mode, simulate lsim for that segment
segments = [1; find(diff(mode_history)~=0)+1; N+1];
for s = 1:(length(segments)-1)
    i1 = segments(s);
    i2 = segments(s+1)-1;
    tt = t(i1:i2);
    ee = e(i1:i2);
    if mode_history(i1) == 1
        % 2DoF: use C_2dof_y (feedback path controller)
        try
            uu = lsim(C_2dof_y, ee, tt);
        catch
            % fallback: simple P action using DC gain
            K = dcgain(C_2dof_y); uu = K * ee;
        end
    else
        % PID
        try
            uu = lsim(C_pid, ee, tt);
        catch
            K = dcgain(C_pid); uu = K * ee;
        end
    end
    u(i1:i2) = uu;
end

end
