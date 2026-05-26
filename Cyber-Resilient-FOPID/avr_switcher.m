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
%    .detector_attack_flag (optional bool): if true, force switch at detector_attack_time
%    .detector_attack_time (optional float): detection timestamp used for the forced switch
%
% Outputs:
%  u: N×1 control signal (controller output)
%  mode_history: N×1 integer mode (1=normal,2=attacking,3=recovery)
%  switch_times: Mx3 [time, from_mode, to_mode]

if nargin < 6 || isempty(switcher_config), switcher_config = struct(); end
if ~isfield(switcher_config,'hysteresis_time'), switcher_config.hysteresis_time = 2.0; end
if ~isfield(switcher_config,'recovery_time'), switcher_config.recovery_time = 0.5; end
if ~isfield(switcher_config,'initial_mode'), switcher_config.initial_mode = 1; end
if ~isfield(switcher_config,'detector_attack_flag'), switcher_config.detector_attack_flag = false; end
if ~isfield(switcher_config,'detector_attack_time'), switcher_config.detector_attack_time = NaN; end
if ~isfield(switcher_config,'persist_state_history'), switcher_config.persist_state_history = false; end

t = t(:); y_meas = y_meas(:); r_ref = r_ref(:);
N = length(t);
if length(y_meas) ~= N || length(r_ref) ~= N
    error('t, y_meas, and r_ref must have same length');
end

% Prepare Phase 4 log for this switcher invocation
outdir4 = fullfile('results','phase4'); if ~exist(outdir4,'dir'), mkdir(outdir4); end
run_ts = datestr(now,'yyyymmdd_HHMMSS');
logpath4 = fullfile(outdir4, sprintf('phase4_switcher_%s.log', run_ts));
lf = fopen(logpath4,'w');
if lf > 0
    fprintf(lf, 'AVR_SWITCHER log - %s\n', datestr(now));
    fprintf(lf, 'hysteresis_time=%.3f, recovery_time=%.3f, initial_mode=%d\n', switcher_config.hysteresis_time, switcher_config.recovery_time, switcher_config.initial_mode);
    if isfield(switcher_config,'detector_attack_flag'), fprintf(lf, 'detector_attack_flag=%d\n', double(switcher_config.detector_attack_flag)); end
    if isfield(switcher_config,'detector_attack_time'), fprintf(lf, 'detector_attack_time=%s\n', num2str(switcher_config.detector_attack_time)); end
    fprintf('Phase4 switcher log: %s\n', logpath4);
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

use_detector_hint = logical(switcher_config.detector_attack_flag) && isfinite(switcher_config.detector_attack_time);
detector_switch_done = false;

for k = 1:N
    % Mode transitions
    if use_detector_hint && ~detector_switch_done
        if t(k) >= switcher_config.detector_attack_time
            if mode ~= 2
                from_mode = mode; mode = 2; switch_times(end+1,:) = [t(k), from_mode, mode]; last_switch_time = t(k);
            end
            detector_switch_done = true;
        end
    elseif mode == 1
        if metric(k) > metric_thresh
            from_mode = mode; mode = 2; switch_times(end+1,:) = [t(k), from_mode, mode]; last_switch_time = t(k);
        end
    end

    if mode == 2
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
% To implement bumpless transfer we simulate both controllers in state-space
% continuously and switch outputs while preserving internal states.
% Use safe conversion to avoid improper TF -> SS errors
ss_c2 = safe_controller_ss(C_2dof_y);
ss_pid = safe_controller_ss(C_pid);

% Extract state-space matrices (handle empty controllers)
if isempty(ss_c2)
    A2 = []; B2 = []; C2 = []; D2 = [];
else
    [A2,B2,C2,D2] = ssdata(ss_c2);
end
if isempty(ss_pid)
    Ap = []; Bp = []; Cp = []; Dp = [];
else
    [Ap,Bp,Cp,Dp] = ssdata(ss_pid);
end

% Initialize controller states
n2 = 0; np = 0;
if ~isempty(A2), n2 = size(A2,1); x2 = zeros(n2,1); else x2 = [] ; end
if ~isempty(Ap), np = size(Ap,1); xp = zeros(np,1); else xp = []; end

% prepare state dumps array for switch events
state_dumps = [];
% optionally preallocate full state histories
if switcher_config.persist_state_history
    xp_hist = zeros(N, max(1,np));
    x2_hist = zeros(N, max(1,n2));
else
    xp_hist = [];
    x2_hist = [];
end

% previous output for bumpless target
u_prev = 0;

% Main time-stepping simulation computing controller states and outputs
for k = 1:N
    if k == 1
        dt = t(1);
    else
        dt = t(k) - t(k-1);
    end

    ek = e(k);

    % update both controllers' internal states and outputs
    if ~isempty(A2)
        x2_dot = A2 * x2 + B2 * ek;
        y2 = C2 * x2 + D2 * ek;
    else
        x2_dot = [];
        y2 = 0;
    end
    if ~isempty(Ap)
        xp_dot = Ap * xp + Bp * ek;
        yp = Cp * xp + Dp * ek;
    else
        xp_dot = [];
        yp = 0;
    end

    % Determine mode transitions (detector hint takes precedence)
    if use_detector_hint && ~detector_switch_done
        if t(k) >= switcher_config.detector_attack_time
                if mode ~= 2
                from_mode = mode; mode = 2; switch_times(end+1,:) = [t(k), from_mode, mode]; last_switch_time = t(k);
                % perform bumpless adjustment: set xp so that yp matches previous u_prev
                if ~isempty(Cp) && any(Cp(:))
                    try
                        xp = pinv(Cp) * (u_prev - Dp * ek);
                        if exist('lf','var') && lf>0, fprintf(lf,'Performed bumpless adjust on PID at t=%.4f (detector switch)\n', t(k)); end
                    catch ME
                        if exist('lf','var') && lf>0, fprintf(lf,'Bumpless adjust (PID) failed at t=%.4f: %s\n', t(k), ME.message); end
                    end
                    % capture state dump
                    sd.time = t(k); sd.from = from_mode; sd.to = mode; sd.xp = xp; sd.x2 = x2;
                    state_dumps = [state_dumps; sd];
                end
            end
            detector_switch_done = true;
        end
    else
        % use metric-based switching as before
            if mode == 1 && metric(k) > metric_thresh
            from_mode = mode; mode = 2; switch_times(end+1,:) = [t(k), from_mode, mode]; last_switch_time = t(k);
            % adjust PID state to avoid jump
            if ~isempty(Cp) && any(Cp(:))
                    try
                        xp = pinv(Cp) * (y2 - Dp * ek);
                        if exist('lf','var') && lf>0, fprintf(lf,'Performed bumpless adjust on PID at t=%.4f (metric switch)\n', t(k)); end
                    catch ME
                        if exist('lf','var') && lf>0, fprintf(lf,'Bumpless adjust (PID) failed at t=%.4f: %s\n', t(k), ME.message); end
                    end
            end
            % capture state dump
            sd.time = t(k); sd.from = from_mode; sd.to = mode; sd.xp = xp; sd.x2 = x2;
            state_dumps = [state_dumps; sd];
        elseif mode == 2
            if t(k) - last_switch_time >= switcher_config.recovery_time
                recent_metric = mean(metric(max(1,k-win):k));
                if recent_metric < metric_thresh/2
                    from_mode = mode; mode = 3; switch_times(end+1,:) = [t(k), from_mode, mode]; last_switch_time = t(k);
                end
            end
        elseif mode == 3
            if t(k) - last_switch_time >= switcher_config.hysteresis_time
                recent_metric = mean(metric(max(1,k-win):k));
                if recent_metric < metric_thresh/2
                    from_mode = mode; mode = 1; switch_times(end+1,:) = [t(k), from_mode, mode]; last_switch_time = t(k);
                    % adjust 2DoF state to align outputs
                            if ~isempty(C2) && any(C2(:))
                        try
                            x2 = pinv(C2) * (yp - D2 * ek);
                            if exist('lf','var') && lf>0, fprintf(lf,'Performed bumpless adjust on 2DoF at t=%.4f (recovery->normal)\n', t(k)); end
                        catch ME
                            if exist('lf','var') && lf>0, fprintf(lf,'Bumpless adjust (2DoF) failed at t=%.4f: %s\n', t(k), ME.message); end
                        end
                    end
                            % capture state dump after recovery->normal
                            sd.time = t(k); sd.from = from_mode; sd.to = mode; sd.xp = xp; sd.x2 = x2;
                            state_dumps = [state_dumps; sd];
                elseif recent_metric > metric_thresh
                    from_mode = mode; mode = 2; switch_times(end+1,:) = [t(k), from_mode, mode]; last_switch_time = t(k);
                end
            end
        end
    end

    % Now compute outputs: use active mode's output, but update states after computing output
    if mode == 1
        u_k = y2;
    else
        u_k = yp;
    end

    u(k) = u_k;
    mode_history(k) = mode;
    u_prev = u_k;

    % integrate states (Euler)
    if ~isempty(A2)
        x2 = x2 + x2_dot * dt;
    end
    if ~isempty(Ap)
        xp = xp + xp_dot * dt;
    end
    % store histories if requested
    if switcher_config.persist_state_history
        if ~isempty(xp), xp_hist(k,1:np) = xp'; end
        if ~isempty(x2), x2_hist(k,1:n2) = x2'; end
    end
end

% After simulation, log switch times if logfile open
if exist('lf','var') && lf>0
    fprintf(lf,'\nSwitch events: %d\n', size(switch_times,1));
    for s=1:size(switch_times,1)
        fprintf(lf,'t=%.4f: %d -> %d\n', switch_times(s,1), switch_times(s,2), switch_times(s,3));
    end
    fprintf(lf,'Final mode: %d\n', mode);
    % Print state dumps
    if ~isempty(state_dumps)
        fprintf(lf,'\nState dumps at switch events:\n');
        for s=1:length(state_dumps)
            sd = state_dumps(s);
            try
                xp_str = mat2str(sd.xp',6);
            catch
                xp_str = '<empty>';
            end
            try
                x2_str = mat2str(sd.x2',6);
            catch
                x2_str = '<empty>';
            end
            fprintf(lf,'t=%.4f: %d->%d, xp=%s, x2=%s\n', sd.time, sd.from, sd.to, xp_str, x2_str);
        end
    end
    % save full histories if requested
    if switcher_config.persist_state_history
        try
            fname = fullfile(outdir4, sprintf('phase4_states_%s.mat', run_ts));
            save(fname, 'xp_hist', 'x2_hist', 't', 'switch_times', 'state_dumps');
            fprintf(lf, 'Saved full state history to %s\n', fname);
        catch ME
            fprintf(lf, 'Failed saving full state history: %s\n', ME.message);
        end
    end
    fclose(lf);
end

end

function ss_sys = safe_controller_ss(C)
    % Attempt to convert controller C to state-space.
    % If conversion fails (improper TF), fall back to a static gain SS using a direct low-frequency evaluation.
    try
        if isa(C,'tf') || isa(C,'zpk') || isa(C,'pid') || isa(C,'pidstd')
            tfC = tf(C);
            try
                [num, den] = tfdata(tfC, 'v');
                if ~isempty(num) && ~isempty(den) && numel(num) >= numel(den)
                    k = real(evalfr(tfC, 0));
                    if ~isfinite(k), k = 1; end
                    ss_sys = ss(k);
                    return;
                end
            catch
                % Fall through to the general conversion path.
            end
        end
        ss_sys = ss(C);
        return;
    catch
        try
            k = evalfr(C, 0);
            if ~isfinite(k)
                k = 1;
            end
            k = real(k);
            if isfinite(k)
                ss_sys = ss(k);
                return;
            else
                ss_sys = ss(1);
                return;
            end
        catch
            ss_sys = ss(1);
            return;
        end
    end
end
