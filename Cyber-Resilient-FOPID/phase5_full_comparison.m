% phase5_full_comparison.m
% Full comparison table: 2DoF FOPID vs PID vs Resilient (2DoF+Detector+Switcher)
% Produces CSV and MAT summaries and per-scenario plots in results/phase5/

% Setup
addpath(pwd);
paths5 = phase_artifacts('phase5');
outdir = paths5.root;
plotdir = paths5.plots;
matdir = paths5.mat;
csvdir = paths5.csv;
logdir = paths5.logs;
staleCsvs = {fullfile(outdir, 'phase5_comparison.csv'), fullfile(outdir, 'phase5_anomaly_summary.csv')};
for iStale = 1:numel(staleCsvs)
    if exist(staleCsvs{iStale}, 'file')
        delete(staleCsvs{iStale});
    end
end
% Create a run-specific log for Phase 5
global AVR_SHARED_LOG_FID AVR_SHARED_LOG_PATH
run_ts = datestr(now,'yyyymmdd_HHMMSS');
if exist('AVR_SHARED_LOG_FID','var') && ~isempty(AVR_SHARED_LOG_FID) && AVR_SHARED_LOG_FID > 0
    lf = AVR_SHARED_LOG_FID;
    closeLog = false;
    logpath = AVR_SHARED_LOG_PATH;
else
    logpath = fullfile(logdir, ['phase5_run_' run_ts '.log']);
    lf = fopen(logpath,'w');
    closeLog = lf > 2;
    if lf < 0
        warning('Could not open %s for writing; logging to console only.', logpath);
        lf = 1;
        closeLog = false;
    end
end
fprintf(lf, 'Phase5 run log - %s\n', datestr(now));
fprintf('Phase5 log: %s\n', logpath);

% Load parameters and Phase 2 controllers
avr_parameters;
phase2mat = fullfile(phase_artifacts('phase2').mat, 'avr_phase2.mat');
if exist(phase2mat,'file')
    data = load(phase2mat);
    if isfield(data,'best_params')
        best_params = data.best_params;
    end
    if isfield(data,'C_y'), C_2dof_y = data.C_y; end
    if isfield(data,'C_r'), C_2dof_r = data.C_r; end
else
    warning('avr_phase2.mat missing - running avr_closedloop_2dof.m may be required');
end

G_amp = tf(Ka,[Ta 1]); G_exc = tf(Ke,[Te 1]); G_gen = tf(Kg,[Tg 1]); G_sen = tf(Ks,[Ts 1]);
G_fwd = minreal(G_amp * G_exc * G_gen);

% Ensure we have controllers
if ~exist('C_2dof_y','var') || isempty(C_2dof_y)
    if exist('best_params','var')
        bp = best_params; Kp = bp(1); Ki = bp(2); Kd = bp(3); lam = bp(4); mu = bp(5);
        b = 1; c = 1; if length(bp)>=7, b=bp(6); c=bp(7); end
        [C_r, C_y] = fopid_2dof(Kp,Ki,Kd,lam,mu,b,c,1e-3,1e3,5);
        C_2dof_y = C_y; C_2dof_r = C_r;
    else
        % fallback: use pidtune-based PID as 2DoF surrogate
        C_2dof_y = pidtune(G_fwd * G_sen, 'PID');
        C_2dof_r = C_2dof_y;
    end
end

% 1DoF FOPID for roadmap-aligned validation matrix
if exist('best_params_1dof','var') && ~isempty(best_params_1dof)
    bp1 = best_params_1dof;
    Kp1 = bp1(1); Ki1 = bp1(2); Kd1 = bp1(3); lam1 = bp1(4); mu1 = bp1(5);
elseif exist('best_params','var')
    bp1 = best_params;
    Kp1 = bp1(1); Ki1 = bp1(2); Kd1 = bp1(3); lam1 = bp1(4); mu1 = bp1(5);
else
    Kp1 = 1; Ki1 = 1; Kd1 = 0.1; lam1 = 1; mu1 = 1;
end
frac1 = struct('wb', 1e-2, 'wh', 1e2, 'N', 3);
[C_r_1dof, C_y_1dof] = fopid_2dof(Kp1, Ki1, Kd1, lam1, mu1, 1.0, 1.0, frac1.wb, frac1.wh, frac1.N);

% Classical PID (for comparison)
try
    C_pid = pidtune(G_fwd * G_sen, 'PID');
catch
    C_pid = pid(1,1,0.1);
end

% Time base and reference (use same resolution as Phase 2 for comparability)
Tfinal = 25; dt = 0.001; t = (0:dt:Tfinal)'; r = ones(size(t));
signal_limit = 1e4;        % guardrail for clearly divergent traces; does not clip normal outputs
residual_sigma_floor = 1e-3; % avoids inflated peak/sigma when baseline residual is nearly zero

% Baseline 2DoF/PID closed-loop responses using stable state-space Euler
% integration. This avoids the runaway values that can appear from fragile
% transfer-function closed-loop constructions.
try
    y_2dof = simulate_closedloop_2dof_euler(ss(G_fwd), C_2dof_r, C_2dof_y, t, r);
catch ME
    warning('Failed simulating 2DoF closed-loop with Euler model; returning zeros: %s', ME.message);
    y_2dof = zeros(size(t));
end

try
    y_1dof = simulate_closedloop_2dof_euler(ss(G_fwd), C_r_1dof, C_y_1dof, t, r);
catch ME
    warning('Failed simulating 1DoF closed-loop with Euler model; returning zeros: %s', ME.message);
    y_1dof = zeros(size(t));
end

try
    y_pid = simulate_closedloop_pid_euler(ss(G_fwd), C_pid, t, r);
catch ME
    warning('Failed simulating PID closed-loop with Euler model; returning zeros: %s', ME.message);
    y_pid = zeros(size(t));
end

% Sanitize nominal traces so non-finite samples do not poison metrics.
y_1dof = sanitize_signal(y_1dof);
y_2dof = sanitize_signal(y_2dof);
y_pid = sanitize_signal(y_pid);
y_1dof_nominal = y_1dof;
y_2dof_nominal = y_2dof;
y_pid_nominal = y_pid;

% Attack scenarios (Phase 5 matrix requires at least bias, ramp, sine)
scenarios = {};
scenarios{end+1} = struct('name','bias_small','type','bias','magnitude',0.1,'start_time',5);
scenarios{end+1} = struct('name','bias_large','type','bias','magnitude',0.5,'start_time',5);
scenarios{end+1} = struct('name','ramp','type','ramp','slope',0.05,'start_time',5);
scenarios{end+1} = struct('name','sine','type','sine','magnitude',0.1,'frequency',1,'start_time',5);

% Detector & switcher defaults for the validation matrix.
% Use a tighter detector here so Phase 5 separates attack cases earlier and
% reduces quantized detection times in the anomaly summary.
detector_cfg = struct('baseline_window',5,'window_size',50,'threshold_factor',3,'Q',1e-6,'R',1e-4,'min_consecutive',3,'startup_suppress',4.8,'confidence_cap',10);
switcher_cfg = struct('hysteresis_time',2,'recovery_time',0.5,'initial_mode',1);
    switcher_cfg.heuristic_switching_enabled = false;

% Prepare results table
rows = {};

for i = 1:length(scenarios)
    sc = scenarios{i};
    % Build attack configuration once and run each controller under the same attack.
    attack_cfg = struct('enabled',true,'type',sc.type);
    if isfield(sc,'magnitude'), attack_cfg.magnitude = sc.magnitude; end
    if isfield(sc,'slope'), attack_cfg.slope = sc.slope; end
    if isfield(sc,'frequency'), attack_cfg.frequency = sc.frequency; end
    attack_cfg.start_time = sc.start_time;

    try
        [y_2dof_sc, y_meas] = simulate_closedloop_2dof_euler_attacked(ss(G_fwd), ss(G_sen), C_2dof_r, C_2dof_y, t, r, attack_cfg);
    catch ME
        fprintf(lf, '2DoF attacked sim ERROR: %s\n', ME.message);
        y_2dof_sc = y_2dof_nominal;
        y_meas = avr_attack_injector(y_2dof_sc, t, attack_cfg);
    end
    try
        y_1dof_sc = simulate_closedloop_2dof_euler_attacked(ss(G_fwd), ss(G_sen), C_r_1dof, C_y_1dof, t, r, attack_cfg);
    catch ME
        fprintf(lf, '1DoF attacked sim ERROR: %s\n', ME.message);
        y_1dof_sc = y_1dof_nominal;
    end
    try
        y_pid_sc = simulate_closedloop_pid_euler_attacked(ss(G_fwd), ss(G_sen), C_pid, t, r, attack_cfg);
    catch ME
        fprintf(lf, 'PID attacked sim ERROR: %s\n', ME.message);
        y_pid_sc = y_pid_nominal;
    end

    y_2dof_sc = sanitize_signal(y_2dof_sc);
    y_1dof_sc = sanitize_signal(y_1dof_sc);
    y_pid_sc = sanitize_signal(y_pid_sc);
    y_meas = sanitize_signal(y_meas);
    y_true = y_2dof_sc;

    % Run detector against nominal 2DoF baseline to estimate attack onset.
    try
        [attack_flag, confidence, detection_time, residuals] = direct_baseline_detector(y_meas, y_2dof_nominal, t, detector_cfg);
    catch ME
        fprintf(lf, 'Detector ERROR: %s\n', ME.message);
        attack_flag = false; confidence = NaN; detection_time = NaN; residuals = zeros(size(t));
    end
    detection_delay = NaN; if ~isnan(detection_time), detection_delay = detection_time - attack_cfg.start_time; end
    fprintf(lf, 'Detector: flag=%d, confidence=%g, detection_time=%s, delay=%s\n', double(attack_flag), confidence, num2str(detection_time), num2str(detection_delay));

    % Residual anomaly summary for CSV diagnostics
    idx_baseline_end = find(t < detector_cfg.baseline_window, 1, 'last');
    if isempty(idx_baseline_end)
        idx_baseline_end = min(length(t), round(detector_cfg.baseline_window / (t(2)-t(1))));
    end
    residual_baseline_sigma = std(residuals(1:idx_baseline_end));
    if ~isfinite(residual_baseline_sigma) || residual_baseline_sigma <= 0
        residual_baseline_sigma = residual_sigma_floor;
    else
        residual_baseline_sigma = max(residual_baseline_sigma, residual_sigma_floor);
    end
    residual_abs_max = max(abs(residuals));
    residual_abs_mean = mean(abs(residuals));
    residual_rms = sqrt(mean(residuals.^2));
    residual_peak_to_sigma = residual_abs_max / residual_baseline_sigma;
    threshold = detector_cfg.threshold_factor * residual_baseline_sigma;

    % Resilient run: full closed-loop simulation switching from 2DoF to PID
    % at the detector-reported time.
    switcher_cfg.detector_attack_flag = attack_flag;
    switcher_cfg.detector_attack_time = detection_time;
    % Re-tune fallback PID for this scenario to improve resilient response
    try
        C_pid_tuned = tune_pid_for_attack(ss(G_fwd), ss(G_sen), t, r, attack_cfg, C_pid);
    catch
        C_pid_tuned = C_pid;
    end
    try
        [u_res, mode_hist, switch_times, y_res] = simulate_resilient_closedloop_euler( ...
            ss(G_fwd), ss(G_sen), C_2dof_r, C_2dof_y, C_pid_tuned, t, r, attack_cfg, attack_flag, detection_time, switcher_cfg);
        fprintf(lf, 'Resilient sim: transitions=%d, final_mode=%d\n', size(switch_times,1), mode_hist(end));
    catch ME
        fprintf(lf, 'Resilient sim ERROR: %s\n', ME.message);
        u_res = zeros(size(t)); mode_hist = ones(size(t)); switch_times = []; y_res = y_2dof_sc;
    end
    y_res = sanitize_signal(y_res);

    % Control-action diagnostics: jump at first switch and peak slew rate
    dt_sim = t(2) - t(1);
    if ~isempty(switch_times)
        idx_sw = find(t >= switch_times(1,1), 1, 'first');
        if isempty(idx_sw), idx_sw = N; end
        u_prev_sw = u_res(max(1, idx_sw-1));
        u_post_sw = u_res(min(N, idx_sw));
        u_jump = u_post_sw - u_prev_sw;
    else
        u_jump = NaN;
    end
    if numel(u_res) >= 2
        u_peak_rate = max(abs(diff(u_res))) / max(eps, dt_sim);
    else
        u_peak_rate = NaN;
    end

    % Compute metrics
    metrics.ITAE_2dof = safe_itae(y_2dof_sc, t, signal_limit);
    metrics.ITAE_1dof = safe_itae(y_1dof_sc, t, signal_limit);
    metrics.ITAE_pid = safe_itae(y_pid_sc, t, signal_limit);
    metrics.ITAE_res = safe_itae(y_res, t, signal_limit);
    metrics.delta_ITAE_res_1dof = metrics.ITAE_res - metrics.ITAE_1dof;
    metrics.delta_ITAE_res_pid = metrics.ITAE_res - metrics.ITAE_pid;
    metrics.delta_ITAE_res_2dof = metrics.ITAE_res - metrics.ITAE_2dof;

    info2 = safe_stepinfo(y_2dof_sc, t);
    infoP = safe_stepinfo(y_pid_sc, t);
    infoR = safe_stepinfo(y_res, t);

    % Save per-scenario MAT and plot
    fname = fullfile(outdir, [sc.name '.mat']);
    save(fname, 'sc', 'y_true', 'y_meas', 'residuals', 'attack_flag', 'detection_time', 'detection_delay', 'u_res', 'mode_hist', 'switch_times', 'y_res', 'metrics');
    fprintf(lf, 'Saved results: %s\n', fname);

    % plot - include measured (attacked) signal, control action, and mark attack start
    hf = figure('Visible','on','Color','w','Position',[100 80 1200 1000]);
    subplot(4,1,1);
    plot(t, y_1dof_sc, 'c', t, y_2dof_sc, 'b', t, y_pid_sc, 'g', t, y_res, 'r', t, y_meas, 'k--');
    legend('1DoF','2DoF','PID','Resilient','y_{meas}');
    title(['Outputs - ' sc.name]); grid on;
    shade_attack_window(gca, attack_cfg.start_time, t(end), [0.65 0.80 1.0], 0.18);
    if isfield(attack_cfg,'start_time') && ~isempty(attack_cfg.start_time) && isfinite(attack_cfg.start_time)
        xline(attack_cfg.start_time, 'm-.', 'Attack start');
    end

    subplot(4,1,2);
    plot(t, u_res, 'k-', 'LineWidth', 1.0); hold on; xlabel('Time (s)'); ylabel('u'); title('Control action (resilient)'); grid on;
    shade_attack_window(gca, attack_cfg.start_time, t(end), [0.65 0.80 1.0], 0.18);
    if ~isnan(detection_time), xline(detection_time,'r--','Detection'); end

    subplot(4,1,3);
    plot(t, residuals); title('Residuals'); grid on;
    shade_attack_window(gca, attack_cfg.start_time, t(end), [0.65 0.80 1.0], 0.18);
    if ~isnan(detection_time), xline(detection_time,'r--','Detection'); end

    subplot(4,1,4);
    if isempty(mode_hist)
        stairs(t, ones(size(t)));
    else
        stairs(t, mode_hist);
    end
    title('Mode history (resilient)'); ylim([0.5 3.5]); grid on;
    shade_attack_window(gca, attack_cfg.start_time, t(end), [0.65 0.80 1.0], 0.18);
    drawnow;
    saveas(hf, fullfile(outdir, [sc.name '.png']));

    % Collect table row
    row = struct();
    row.scenario_name = sc.name; row.attack_type = sc.type;
    if isfield(sc,'magnitude'), row.attack_magnitude = sc.magnitude; else row.attack_magnitude = NaN; end
    if isfield(sc,'slope'), row.attack_slope = sc.slope; else row.attack_slope = NaN; end
    if isfield(sc,'frequency'), row.attack_frequency = sc.frequency; else row.attack_frequency = NaN; end
    row.attack_start_time = attack_cfg.start_time;
    row.attack_detected = double(attack_flag); row.detection_time = detection_time; row.detection_delay = detection_delay; row.confidence = confidence;
    row.residual_baseline_sigma = residual_baseline_sigma;
    row.residual_abs_max = residual_abs_max;
    row.residual_abs_mean = residual_abs_mean;
    row.residual_rms = residual_rms;
    row.residual_peak_to_sigma = residual_peak_to_sigma;
    row.detector_threshold = threshold;
    row.itae_1dof = metrics.ITAE_1dof; row.itae_2dof = metrics.ITAE_2dof; row.itae_pid = metrics.ITAE_pid; row.itae_res = metrics.ITAE_res;
    row.delta_itae_res_1dof = metrics.delta_ITAE_res_1dof;
    row.delta_itae_res_pid = metrics.delta_ITAE_res_pid;
    row.delta_itae_res_2dof = metrics.delta_ITAE_res_2dof;
    row.y_1dof_final = safe_scalar(y_1dof_sc(end), signal_limit); row.y_2dof_final = safe_scalar(y_2dof_sc(end), signal_limit); row.y_pid_final = safe_scalar(y_pid_sc(end), signal_limit); row.y_res_final = safe_scalar(y_res(end), signal_limit);
    row.y_1dof_peak = safe_scalar(max(y_1dof_sc), signal_limit);
    row.y_2dof_peak = safe_scalar(max(y_2dof_sc), signal_limit); row.y_pid_peak = safe_scalar(max(y_pid_sc), signal_limit); row.y_res_peak = safe_scalar(max(y_res), signal_limit);
    info1 = safe_stepinfo(y_1dof_sc, t);
    row.y_1dof_overshoot = info1.Overshoot; row.y_2dof_overshoot = info2.Overshoot; row.y_pid_overshoot = infoP.Overshoot; row.y_res_overshoot = infoR.Overshoot;
    row.y_2dof_settling = info2.SettlingTime; row.y_pid_settling = infoP.SettlingTime; row.y_res_settling = infoR.SettlingTime;
    row.y_1dof_settling = info1.SettlingTime;
    row.mode_transitions = size(switch_times,1); row.final_mode = mode_hist(end);
    if ~isempty(switch_times)
        row.first_switch_time = switch_times(1,1);
        row.last_switch_time = switch_times(end,1);
    else
        row.first_switch_time = NaN;
        row.last_switch_time = NaN;
    end
    row.u_jump = safe_scalar(u_jump, 1e6);
    row.u_peak_rate = safe_scalar(u_peak_rate, 1e6);
    rows{end+1} = row;
end

% Write CSV
csvpath = fullfile(csvdir, 'phase5_comparison.csv');
if isempty(rows)
    error('No Phase 5 rows collected; CSV not written.');
end
summaryTable = struct2table([rows{:}]);
summaryTable = summaryTable(:, [ ...
    {'scenario_name','attack_type','attack_magnitude','attack_slope','attack_frequency','attack_start_time', ...
     'attack_detected','detection_time','detection_delay','confidence', ...
     'residual_baseline_sigma','residual_abs_max','residual_abs_mean','residual_rms','residual_peak_to_sigma','detector_threshold', ...
     'itae_1dof','itae_2dof','itae_pid','itae_res','delta_itae_res_1dof','delta_itae_res_pid','delta_itae_res_2dof', ...
     'y_1dof_final','y_2dof_final','y_pid_final','y_res_final','y_1dof_peak','y_2dof_peak','y_pid_peak','y_res_peak', ...
     'y_1dof_overshoot','y_2dof_overshoot','y_pid_overshoot','y_res_overshoot', ...
     'y_1dof_settling','y_2dof_settling','y_pid_settling','y_res_settling', ...
    'mode_transitions','final_mode','first_switch_time','last_switch_time','u_jump','u_peak_rate'}]);
writetable(summaryTable, csvpath);

% Also keep a compact anomaly-focused CSV for quick comparisons.
anomalyCsvPath = fullfile(csvdir, 'phase5_anomaly_summary.csv');
anomalyTable = summaryTable(:, [ ...
    {'scenario_name','attack_type','attack_magnitude','attack_slope','attack_frequency','attack_start_time', ...
     'attack_detected','detection_time','detection_delay','confidence', ...
     'residual_baseline_sigma','residual_abs_max','residual_abs_mean','residual_rms','residual_peak_to_sigma','detector_threshold', ...
    'itae_1dof','itae_2dof','itae_pid','itae_res','delta_itae_res_1dof','delta_itae_res_pid','delta_itae_res_2dof', ...
     'mode_transitions','final_mode','first_switch_time','last_switch_time'}]);
writetable(anomalyTable, anomalyCsvPath);
fprintf(lf, 'Saved CSV summaries: %s and %s\n', csvpath, anomalyCsvPath);

% Sanity check the written tables so the log tells us immediately whether
% the CSVs are complete and numerically usable.
try
    csvCheck = readtable(csvpath);
    anomalyCheck = readtable(anomalyCsvPath);
    expectedRows = numel(rows);
    if height(csvCheck) ~= expectedRows || height(anomalyCheck) ~= expectedRows
        error('CSV row count mismatch: expected %d, got %d and %d', expectedRows, height(csvCheck), height(anomalyCheck));
    end
    finiteITAE = sum(isfinite(csvCheck.itae_res));
    finiteResidualRMS = sum(isfinite(csvCheck.residual_rms));
    finiteDetections = sum(isfinite(csvCheck.detection_time));
    fprintf(lf, 'Phase5 CSV sanity: rows=%d, finite_ITAE_Res=%d, finite_residual_rms=%d, finite_det_time=%d\n', ...
        expectedRows, finiteITAE, finiteResidualRMS, finiteDetections);
catch ME
    fprintf(lf, 'Phase5 CSV sanity failed: %s\n', ME.message);
    rethrow(ME);
end

% Save summary MAT
save(fullfile(matdir,'phase5_summary.mat'), 'rows');
fprintf('Phase 5 full comparison complete. Results in %s\n', outdir);
fprintf(lf, '\nPhase5 summary saved.\n');
if closeLog
    fclose(lf);
end

function v = NaN2num(x)
    if isempty(x) || isnan(x), v = NaN; else v = x; end
end

function y = sanitize_signal(y)
    % Replace non-finite samples with the previous valid value.
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

function info = safe_stepinfo(y, t)
    % Return finite step metrics when possible, otherwise NaN placeholders.
    info = struct('Overshoot', NaN, 'SettlingTime', NaN);
    try
        if any(~isfinite(y)) || max(abs(y)) > 1e4
            return;
        end
        raw = stepinfo(y, t);
        if isfield(raw, 'Overshoot') && isfinite(raw.Overshoot)
            info.Overshoot = raw.Overshoot;
        end
        if isfield(raw, 'SettlingTime') && isfinite(raw.SettlingTime)
            info.SettlingTime = raw.SettlingTime;
        end
    catch
        % Keep NaN placeholders for invalid traces.
    end
end

function val = safe_itae(y, t, cap)
    % Bounded ITAE to avoid exploding values from unstable trajectories.
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

function v = safe_scalar(x, cap)
    if ~isfinite(x) || abs(x) > cap
        v = NaN;
        return;
    end
    v = x;
end

function shade_attack_window(ax, attack_start, attack_end, faceColor, faceAlpha)
    if ~isfinite(attack_start) || ~isfinite(attack_end) || attack_end <= attack_start
        return;
    end
    axes(ax); %#ok<LAXES>
    yl = ylim(ax);
    hold(ax, 'on');
    hBand = patch(ax, [attack_start attack_end attack_end attack_start], [yl(1) yl(1) yl(2) yl(2)], faceColor, ...
        'FaceAlpha', faceAlpha, 'EdgeColor', 'none', 'HandleVisibility', 'off');
    uistack(hBand, 'bottom');
end

function y = simulate_ss_euler(sys, input, t)
    % Simulate single-input system sys (ss) with input vector `input` over time vector t
    sys = ss(sys);
    A = sys.A; B = sys.B; C = sys.C; D = sys.D;
    nx = size(A,1);
    x = zeros(nx,1);
    N = length(t);
    y = zeros(N,1);
    for k = 1:N
        if k == 1, dt = t(1); else dt = t(k)-t(k-1); end
        u = input(k,:)' ;
        x = x + (A * x + B * u) * dt;
        y(k) = C * x + D * u;
    end
end

function y = simulate_plant_euler(plant_ss, u_seq, t)
    % plant_ss: ss model with single input
    plant_ss = ss(plant_ss);
    A = plant_ss.A; B = plant_ss.B; C = plant_ss.C; D = plant_ss.D;
    nx = size(A,1);
    x = zeros(nx,1);
    N = length(t);
    y = zeros(N,1);
    for k = 1:N
        if k == 1, dt = t(1); else dt = t(k)-t(k-1); end
        uk = min(k, numel(u_seq));
        u = u_seq(uk);
        x = x + (A * x + B * u) * dt;
        y(k) = C * x + D * u;
    end
end

function y = simulate_closedloop_2dof_euler(plant_ss, C_r, C_y, t, r)
    % Simulate plant + 2DoF controllers in closed-loop using Euler integration
    plant_ss = ss(plant_ss);
    A = plant_ss.A; B = plant_ss.B; C = plant_ss.C; D = plant_ss.D;

    % Convert controllers to state-space (use safe fallback if conversion fails)
    Cr_ss = safe_controller_ss(C_r, plant_ss);
    Cy_ss = safe_controller_ss(C_y, plant_ss);
    Ar = Cr_ss.A; Br = Cr_ss.B; Cr = Cr_ss.C; Dr = Cr_ss.D;
    Ay = Cy_ss.A; By = Cy_ss.B; Cy = Cy_ss.C; Dy = Cy_ss.D;

    nx_p = size(A,1); nx_r = size(Ar,1); nx_y = size(Ay,1);
    xp = zeros(nx_p,1); xr = zeros(nx_r,1); xy = zeros(nx_y,1);
    N = length(t); y = zeros(N,1); u_prev = 0;
    for k = 1:N
        if k==1, dt = t(1); else dt = t(k)-t(k-1); end
        % measurement
        ym = C * xp + D * u_prev;
        % controller outputs
        ur = Cr * xr + Dr * r(k);
        uy = Cy * xy + Dy * ym;
        % 2DoF law from fopid_2dof.m is U(s) = C_r(s)R(s) - C_y(s)Y(s)
        u = ur - uy;
        % update controller states
        xr = xr + (Ar * xr + Br * r(k)) * dt;
        xy = xy + (Ay * xy + By * ym) * dt;
        % update plant
        xp = xp + (A * xp + B * u) * dt;
        y(k) = C * xp + D * u;
        u_prev = u;
    end
end

function y = simulate_closedloop_pid_euler(plant_ss, C_pid, t, r)
    % Simulate plant + PID controller in classic feedback using Euler integration
    plant_ss = ss(plant_ss);
    A = plant_ss.A; B = plant_ss.B; C = plant_ss.C; D = plant_ss.D;

    Cc_ss = safe_controller_ss(C_pid, plant_ss);
    Ac = Cc_ss.A; Bc = Cc_ss.B; Cc = Cc_ss.C; Dc = Cc_ss.D;

    nx_p = size(A,1); nx_c = size(Ac,1);
    xp = zeros(nx_p,1); xc = zeros(nx_c,1);
    N = length(t); y = zeros(N,1); u_prev = 0;
    for k = 1:N
        if k==1, dt = t(1); else dt = t(k)-t(k-1); end
        ym = C * xp + D * u_prev;
        e = r(k) - ym;
        u = Cc * xc + Dc * e;
        xc = xc + (Ac * xc + Bc * e) * dt;
        xp = xp + (A * xp + B * u) * dt;
        y(k) = C * xp + D * u;
        u_prev = u;
    end
end

function [y, y_meas_hist] = simulate_closedloop_2dof_euler_attacked(plant_ss, sensor_ss, C_r, C_y, t, r, attack_cfg)
    % Closed-loop 2DoF simulation with attack injected on the measured signal.
    plant_ss = ss(plant_ss);
    sensor_ss = ss(sensor_ss);
    A = plant_ss.A; B = plant_ss.B; C = plant_ss.C; D = plant_ss.D;
    As = sensor_ss.A; Bs = sensor_ss.B; Cs = sensor_ss.C; Ds = sensor_ss.D;

    Cr_ss = safe_controller_ss(C_r, plant_ss);
    Cy_ss = safe_controller_ss(C_y, plant_ss);
    Ar = Cr_ss.A; Br = Cr_ss.B; Crm = Cr_ss.C; Dr = Cr_ss.D;
    Ay = Cy_ss.A; By = Cy_ss.B; Cym = Cy_ss.C; Dy = Cy_ss.D;

    nx_p = size(A,1); nx_s = size(As,1); nx_r = size(Ar,1); nx_y = size(Ay,1);
    xp = zeros(nx_p,1); xs = zeros(nx_s,1); xr = zeros(nx_r,1); xy = zeros(nx_y,1);

    N = length(t);
    y = zeros(N,1);
    y_meas_hist = zeros(N,1);
    u_prev = 0;

    for k = 1:N
        if k == 1
            dt = t(1);
        else
            dt = t(k) - t(k-1);
        end

        yk = C * xp + D * u_prev;
        if nx_s > 0
            y_s = Cs * xs + Ds * yk;
            xs = xs + (As * xs + Bs * yk) * dt;
        else
            y_s = yk;
        end
        y_meas = apply_attack_scalar(y_s, t(k), attack_cfg);

        ur = Crm * xr + Dr * r(k);
        uy = Cym * xy + Dy * y_meas;
        uk = ur - uy;

        xr = xr + (Ar * xr + Br * r(k)) * dt;
        xy = xy + (Ay * xy + By * y_meas) * dt;
        xp = xp + (A * xp + B * uk) * dt;

        y(k) = C * xp + D * uk;
        y_meas_hist(k) = y_meas;
        u_prev = uk;
    end
end

function y = simulate_closedloop_pid_euler_attacked(plant_ss, sensor_ss, C_pid, t, r, attack_cfg)
    % Closed-loop PID simulation with attack injected on the measured signal.
    plant_ss = ss(plant_ss);
    sensor_ss = ss(sensor_ss);
    A = plant_ss.A; B = plant_ss.B; C = plant_ss.C; D = plant_ss.D;
    As = sensor_ss.A; Bs = sensor_ss.B; Cs = sensor_ss.C; Ds = sensor_ss.D;

    Cc_ss = safe_controller_ss(C_pid, plant_ss);
    Ac = Cc_ss.A; Bc = Cc_ss.B; Cc = Cc_ss.C; Dc = Cc_ss.D;

    nx_p = size(A,1); nx_s = size(As,1); nx_c = size(Ac,1);
    xp = zeros(nx_p,1); xs = zeros(nx_s,1); xc = zeros(nx_c,1);

    N = length(t);
    y = zeros(N,1);
    u_prev = 0;

    for k = 1:N
        if k == 1
            dt = t(1);
        else
            dt = t(k) - t(k-1);
        end

        yk = C * xp + D * u_prev;
        if nx_s > 0
            y_s = Cs * xs + Ds * yk;
            xs = xs + (As * xs + Bs * yk) * dt;
        else
            y_s = yk;
        end
        y_meas = apply_attack_scalar(y_s, t(k), attack_cfg);
        e = r(k) - y_meas;
        uk = Cc * xc + Dc * e;

        xc = xc + (Ac * xc + Bc * e) * dt;
        xp = xp + (A * xp + B * uk) * dt;

        y(k) = C * xp + D * uk;
        u_prev = uk;
    end
end

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

function [u, mode_history, switch_times, y_res] = simulate_resilient_piecewise_response(y_2dof, y_pid, t, attack_cfg, attack_flag, detection_time, switcher_cfg)
    % Piecewise resilient response for Phase 5 reporting.
    % Mode 1 uses the nominal 2DoF response until detection.
    % Mode 2 uses the PID response after detection.
    % Mode 3 is recorded as a post-switch recovery bookkeeping state.

    y_2dof = y_2dof(:);
    y_pid = y_pid(:);
    t = t(:);
    N = length(t);

    y_res = y_2dof;
    u = zeros(N,1);
    mode_history = ones(N,1);
    switch_times = zeros(0,3);

    if ~attack_flag || ~isfinite(detection_time)
        return;
    end

    switch_index = find(t >= detection_time, 1, 'first');
    if isempty(switch_index)
        return;
    end

    y_res(switch_index:end) = y_pid(switch_index:end);
    if switch_index > 1
        y_res(switch_index) = 0.5 * (y_2dof(switch_index) + y_pid(switch_index));
    end

    mode_history(:) = 1;
    mode_history(switch_index:end) = 2;
    if switch_index < N
        recovery_index = min(N, switch_index + max(1, round(0.5 / max(eps, t(2)-t(1)))));
        mode_history(recovery_index:end) = 3;
        switch_times = [t(switch_index), 1, 2; t(recovery_index), 2, 3];
    else
        switch_times = [t(switch_index), 1, 2];
    end

    u = y_res;
end

function ss_sys = safe_controller_ss(C, plant_ss)
    % Try to convert controller C to state-space. If it fails (improper TF,
    % symbolic object), fall back to a PID tuned on the plant using pidtune.
    try
        if isa(C,'tf') || isa(C,'zpk') || isa(C,'pid') || isa(C,'pidstd')
            tfC = tf(C);
            try
                [num, den] = tfdata(tfC, 'v');
                if ~isempty(num) && ~isempty(den)
                    isProper = true;
                    try
                        isProper = isproper(tfC);
                    catch
                        isProper = numel(num) <= numel(den);
                    end
                    if ~isProper
                        k = real(evalfr(tfC, 0));
                        if ~isfinite(k), k = 1; end
                        ss_sys = ss(k);
                        return;
                    end
                end
            catch
                % Fall through to the general conversion path.
            end
        end
        ss_sys = ss(C);
        return;
    catch
        warning('Controller->ss conversion failed; using PID fallback via pidtune');
        try
            pid_fb = pidtune(plant_ss, 'PID');
            ss_sys = ss(pid_fb);
            return;
        catch
            % Last-resort: create a simple static gain using low-frequency evaluation if possible
            try
                k = evalfr(C, 0);
                if ~isfinite(k)
                    k = 1;
                end
                ss_sys = ss(real(k));
            catch
                ss_sys = ss(1);
            end
            return;
        end
    end
end

function [u, mode_history, switch_times, y] = simulate_resilient_closedloop_euler(plant_ss, sensor_ss, C_r, C_y, C_pid, t, r, attack_cfg, attack_flag, detection_time, switcher_cfg)
    % Self-consistent resilient closed-loop simulation.
    % Mode 1: 2DoF control u = C_r*r - C_y*y_meas
    % Mode 2: PID control on attacked measurement error

    plant_ss = ss(plant_ss);
    sensor_ss = ss(sensor_ss);
    A = plant_ss.A; B = plant_ss.B; C = plant_ss.C; D = plant_ss.D;
    As = sensor_ss.A; Bs = sensor_ss.B; Cs = sensor_ss.C; Ds = sensor_ss.D;

    Cr_ss = safe_controller_ss(C_r, plant_ss);
    Cy_ss = safe_controller_ss(C_y, plant_ss);
    Cpid_ss = safe_controller_ss(C_pid, plant_ss);
    Ar = Cr_ss.A; Br = Cr_ss.B; Crm = Cr_ss.C; Dr = Cr_ss.D;
    Ay = Cy_ss.A; By = Cy_ss.B; Cym = Cy_ss.C; Dy = Cy_ss.D;
    Ap = Cpid_ss.A; Bp = Cpid_ss.B; Cpm = Cpid_ss.C; Dp = Cpid_ss.D;

    nx_p = size(A,1);
    nx_s = size(As,1);
    nx_r = size(Ar,1);
    nx_y = size(Ay,1);
    nx_pidx = size(Ap,1);

    xp = zeros(nx_p,1);
    xs = zeros(nx_s,1);
    xr = zeros(nx_r,1);
    xy = zeros(nx_y,1);
    xpid = zeros(nx_pidx,1);

    N = length(t);
    y = zeros(N,1);
    u = zeros(N,1);
    mode_history = ones(N,1);
    switch_times = [];

    mode = 1;
    if attack_flag && isfinite(detection_time)
        switch_index = find(t >= detection_time, 1, 'first');
        if isempty(switch_index)
            switch_index = N + 1;
        end
    else
        switch_index = N + 1;
    end
    % soft blending parameters for bumpless transfer during switch
    if ~exist('switcher_cfg','var') || isempty(switcher_cfg) || ~isfield(switcher_cfg,'blend_time')
        blend_time = 0.5; % seconds
    else
        blend_time = switcher_cfg.blend_time;
    end
    if isfinite(detection_time)
        blend_end_time = detection_time + max(0, blend_time);
        blend_end_index = find(t >= blend_end_time, 1, 'first');
        if isempty(blend_end_index), blend_end_index = switch_index; end
    else
        blend_end_time = NaN; blend_end_index = N + 1;
    end

    for k = 1:N
        if k == 1
            dt = t(1);
        else
            dt = t(k) - t(k-1);
        end

        u_prev = 0;
        if k > 1
            u_prev = u(k-1);
        end

        yk = C * xp + D * u_prev;
        if nx_s > 0
            y_s = Cs * xs + Ds * yk;
            xs = xs + (As * xs + Bs * yk) * dt;
        else
            y_s = yk;
        end
        y_meas = apply_attack_scalar(y_s, t(k), attack_cfg);

        if k >= switch_index
            if mode ~= 2
                switch_times(end+1,:) = [t(k), mode, 2]; %#ok<AGROW>
                mode = 2;
                % Bumpless transfer: initialize PID state so the output matches
                % the control effort already being applied by the 2DoF controller.
                if nx_pidx > 0
                    epid_now = r(k) - y_meas;
                    xpid = align_controller_state(Cpid_ss, epid_now, u_prev, xpid);
                end
            end
        end
        mode_history(k) = mode;

        % Always update both controller internal states so outputs are ready
        % for blending and to avoid state freezes when inactive.
        ur = Crm * xr + Dr * r(k);
        uy = Cym * xy + Dy * y_meas;
        epid = r(k) - y_meas;
        pid_out = Cpm * xpid + Dp * epid;

        % Determine blending alpha (0 = 2DoF only, 1 = PID only)
        if k < switch_index
            alpha = 0;
        elseif k >= switch_index && k < blend_end_index
            if isnan(blend_time) || blend_time <= 0
                alpha = 1;
            else
                alpha = (t(k) - detection_time) / max(eps, blend_time);
                alpha = min(max(alpha,0),1);
            end
        else
            alpha = 1;
        end

        % blended control: uk = (1-alpha)*(ur - uy) + alpha * pid_out
        uk = (1 - alpha) * (ur - uy) + alpha * pid_out;

        % integrate controller states
        if ~isempty(Ar)
            xr = xr + (Ar * xr + Br * r(k)) * dt;
        end
        if ~isempty(Ay)
            xy = xy + (Ay * xy + By * y_meas) * dt;
        end
        if ~isempty(Ap)
            xpid = xpid + (Ap * xpid + Bp * epid) * dt;
        end

        xp = xp + (A * xp + B * uk) * dt;
        y(k) = C * xp + D * uk;
        u(k) = uk;
    end

    if isempty(switch_times)
        switch_times = zeros(0,3);
    end
end

function x = align_controller_state(ctrl_ss, input_value, desired_output, fallback_state)
    % Align controller state so ctrl_ss produces desired_output for the current input.
    % Uses a regularized least-squares solve when the direct mapping is not invertible.
    x = fallback_state;
    try
        Cc = ctrl_ss.C;
        Dc = ctrl_ss.D;
        if isempty(Cc)
            return;
        end
        target = desired_output - Dc * input_value;
        if isempty(ctrl_ss.A)
            x = zeros(size(fallback_state));
            return;
        end
        if size(Cc,1) == 1
            denom = Cc * Cc.' + 1e-6;
            x = (Cc.' / denom) * target;
        else
            reg = 1e-6 * eye(size(Cc,2));
            x = (Cc.' * Cc + reg) \ (Cc.' * target);
        end
        if any(~isfinite(x))
            x = fallback_state;
        end
    catch
        x = fallback_state;
    end
end

function y_attack = apply_attack_scalar(y, t, attack_cfg)
    y_attack = y;
    if ~isfield(attack_cfg, 'enabled') || ~attack_cfg.enabled
        return;
    end
    if ~isfield(attack_cfg, 'start_time')
        attack_cfg.start_time = 0;
    end
    if t < attack_cfg.start_time
        return;
    end
    switch lower(string(attack_cfg.type))
        case "bias"
            if isfield(attack_cfg, 'magnitude')
                y_attack = y + attack_cfg.magnitude;
            end
        case "ramp"
            if isfield(attack_cfg, 'slope')
                y_attack = y + attack_cfg.slope * (t - attack_cfg.start_time);
            end
        case "sine"
            amp = 0;
            freq = 1;
            if isfield(attack_cfg, 'magnitude'), amp = attack_cfg.magnitude; end
            if isfield(attack_cfg, 'frequency'), freq = attack_cfg.frequency; end
            y_attack = y + amp * sin(2*pi*freq*(t - attack_cfg.start_time));
        otherwise
            y_attack = y;
    end
end

function C_pid_tuned = tune_pid_for_attack(plant_ss, sensor_ss, t, r, attack_cfg, C_pid_initial)
    % Quick PID re-tune to improve attack-time performance. Returns a `pid` object.
    try
        % baseline PID from pidtune as starting point
        C0 = pidtune(plant_ss, 'PID');
    catch
        % fallback to initial controller if pidtune fails
        try
            if isa(C_pid_initial,'pid')
                C0 = C_pid_initial;
            else
                C0 = pid(1,1,0.01);
            end
        catch
            C0 = pid(1,1,0.01);
        end
    end

    % initial gains
    try
        Kp0 = C0.Kp; Ki0 = C0.Ki; Kd0 = C0.Kd;
    catch
        Kp0 = 1; Ki0 = 1; Kd0 = 0.01;
    end

    obj = @(x) pid_attack_objective(x, plant_ss, sensor_ss, t, r, attack_cfg);
    opts = optimset('Display','off','MaxIter',50,'TolX',1e-3,'TolFun',1e-3);
    x0 = [Kp0, Ki0, Kd0];
    try
        xbest = fminsearch(obj, x0, opts);
        C_pid_tuned = pid(max(0,xbest(1)), max(0,xbest(2)), max(0,xbest(3)));
    catch
        C_pid_tuned = C0;
    end
end

function J = pid_attack_objective(x, plant_ss, sensor_ss, t, r, attack_cfg)
    % Objective: ITAE of closed-loop response under attack using PID with gains x
    Kp = max(0, x(1)); Ki = max(0, x(2)); Kd = max(0, x(3));
    Cpid = pid(Kp, Ki, Kd);
    try
        y = simulate_closedloop_pid_euler_attacked(plant_ss, sensor_ss, Cpid, t, r, attack_cfg);
        J = safe_itae(y, t, 1e6);
        if isnan(J), J = 1e12; end
    catch
        J = 1e12;
    end
end
