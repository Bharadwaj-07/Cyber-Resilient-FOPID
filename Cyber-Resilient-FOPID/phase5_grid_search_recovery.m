% phase5_grid_search_recovery.m
% Grid-search for resilient switching / recovery parameters.
% Sweeps blend_time, recovery_time, bumpless_reg, actuator_limits and
% evaluates per-scenario metrics (ITAE_res, y_res_final, u_jump, u_peak_rate).

addpath(pwd);
paths5 = phase_artifacts('phase5');
outcsv = fullfile(paths5.csv, 'phase5_grid_search_results.csv');
outmat = fullfile(paths5.mat, 'phase5_grid_search_results.mat');

% Load system and Phase2 controllers
avr_parameters;
phase2mat = fullfile(phase_artifacts('phase2').mat, 'avr_phase2.mat');
if exist(phase2mat,'file')
    data = load(phase2mat);
    if isfield(data,'C_y'), C_2dof_y = data.C_y; end
    if isfield(data,'C_r'), C_2dof_r = data.C_r; end
    if isfield(data,'best_params'), best_params = data.best_params; end
end
G_amp = tf(Ka,[Ta 1]); G_exc = tf(Ke,[Te 1]); G_gen = tf(Kg,[Tg 1]); G_sen = tf(Ks,[Ts 1]);
G_fwd = minreal(G_amp * G_exc * G_gen);

% Fallback controllers
if ~exist('C_2dof_y','var') || isempty(C_2dof_y)
    try, C_2dof_y = pidtune(G_fwd * G_sen, 'PID'); C_2dof_r = C_2dof_y; catch, C_2dof_y = pid(1,1,0.1); C_2dof_r = C_2dof_y; end
% 1DoF fallback
if exist('best_params','var')
    bp1 = best_params; Kp1 = bp1(1); Ki1 = bp1(2); Kd1 = bp1(3); lam1 = bp1(4); mu1 = bp1(5);
else
    Kp1 = 1; Ki1 = 1; Kd1 = 0.1; lam1 = 1; mu1 = 1;
end
frac1 = struct('wb', 1e-2, 'wh', 1e2, 'N', 3);
[C_r_1dof, C_y_1dof] = fopid_2dof(Kp1, Ki1, Kd1, lam1, mu1, 1.0, 1.0, frac1.wb, frac1.wh, frac1.N);
try, C_pid = pidtune(G_fwd * G_sen, 'PID'); catch, C_pid = pid(1,1,0.1); end

% Time base
Tfinal = 25; dt = 0.001; t = (0:dt:Tfinal)'; r = ones(size(t));

% Scenarios
scenarios = {};
scenarios{end+1} = struct('name','bias_small','type','bias','magnitude',0.1,'start_time',5);
scenarios{end+1} = struct('name','bias_large','type','bias','magnitude',0.5,'start_time',5);
scenarios{end+1} = struct('name','ramp','type','ramp','slope',0.05,'start_time',5);
scenarios{end+1} = struct('name','sine','type','sine','magnitude',0.1,'frequency',1,'start_time',5);

% Grid (choose defaults reasonable for local runs)
blend_list = [0.5, 1.0, 1.5, 2.0];
recovery_list = [0.5, 1.0, 2.0];
reg_list = [1e-4, 1e-3, 1e-2];
act_limits = {[-2 2], [-5 5]};

results = [];
row = 0;
total = numel(blend_list)*numel(recovery_list)*numel(reg_list)*numel(act_limits)*numel(scenarios);
fprintf('Grid search combos: %d simulations total\n', total);

for ib = 1:numel(blend_list)
    for ir = 1:numel(recovery_list)
        for ig = 1:numel(reg_list)
            for ia = 1:numel(act_limits)
                cfg = struct();
                cfg.blend_time = blend_list(ib);
                cfg.recovery_time = recovery_list(ir);
                cfg.bumpless_reg = reg_list(ig);
                cfg.actuator_limits = act_limits{ia};
                for is = 1:numel(scenarios)
                    sc = scenarios{is};
                    attack_cfg = struct('enabled',true,'type',sc.type);
                    if isfield(sc,'magnitude'), attack_cfg.magnitude = sc.magnitude; end
                    if isfield(sc,'slope'), attack_cfg.slope = sc.slope; end
                    if isfield(sc,'frequency'), attack_cfg.frequency = sc.frequency; end
                    attack_cfg.start_time = sc.start_time;

                    % Simulate baseline and resilient responses
                    y_2dof_sc = simulate_closedloop_2dof_euler_attacked(ss(G_fwd), ss(G_sen), C_2dof_r, C_2dof_y, t, r, attack_cfg);
                    y_1dof_sc = simulate_closedloop_2dof_euler_attacked(ss(G_fwd), ss(G_sen), C_r_1dof, C_y_1dof, t, r, attack_cfg);
                    y_pid_sc = simulate_closedloop_pid_euler_attacked(ss(G_fwd), ss(G_sen), C_pid, t, r, attack_cfg);

                    % detector (reuse simple baseline detector)
                    [attack_flag, ~, detection_time, residuals] = direct_baseline_detector(y_2dof_sc, y_2dof_sc, t, struct('baseline_window',5,'window_size',50,'threshold_factor',3,'min_consecutive',3,'startup_suppress',4.8));

                    C_pid_tuned = C_pid; % skip fminsearch here for speed

                    [u_res, mode_hist, switch_times, y_res] = simulate_resilient_closedloop_euler( ss(G_fwd), ss(G_sen), C_2dof_r, C_2dof_y, C_pid_tuned, t, r, attack_cfg, attack_flag, detection_time, cfg);

                    y_res = sanitize_signal(y_res);
                    dt_sim = t(2)-t(1);
                    if ~isempty(switch_times)
                        idx_sw = find(t >= switch_times(1,1), 1, 'first'); if isempty(idx_sw), idx_sw = numel(t); end
                        u_prev_sw = u_res(max(1, idx_sw-1)); u_post_sw = u_res(min(numel(u_res), idx_sw)); u_jump = u_post_sw - u_prev_sw;
                    else
                        u_jump = NaN;
                    end
                    if numel(u_res) >= 2, u_peak_rate = max(abs(diff(u_res))) / max(eps, dt_sim); else u_peak_rate = NaN; end

                    itae_res = safe_itae(y_res, t, 1e6);
                    row = row + 1;
                    results(row).scenario = sc.name;
                    results(row).blend_time = cfg.blend_time; results(row).recovery_time = cfg.recovery_time; results(row).bumpless_reg = cfg.bumpless_reg; results(row).actuator_limits = cfg.actuator_limits;
                    results(row).itae_res = itae_res; results(row).y_res_final = safe_scalar(y_res(end),1e6); results(row).u_jump = u_jump; results(row).u_peak_rate = u_peak_rate;
                    if mod(row,10)==0, fprintf('Progress: %d/%d\n', row, total); end
                end
            end
        end
    end
end

% Save results table
T = struct2table(results);
if ~exist(paths5.csv,'dir'), mkdir(paths5.csv); end
writetable(T, outcsv);
save(outmat, 'results', 'T');
fprintf('Grid search complete. Results: %s and %s\n', outcsv, outmat);

% --- helper functions (copied minimal versions from phase5_full_comparison) ---
function y = sanitize_signal(y)
    y = y(:);
    y(~isfinite(y)) = NaN;
    if all(isnan(y)), y = zeros(size(y)); return; end
    firstValid = find(~isnan(y), 1, 'first'); if firstValid > 1, y(1:firstValid-1) = y(firstValid); end
    for k = 2:numel(y), if isnan(y(k)), y(k) = y(k-1); end; end
end

function val = safe_itae(y, t, cap)
    if any(~isfinite(y)) || max(abs(y)) > cap, val = NaN; return; end
    e = abs(1 - y(:)); val = trapz(t(:), t(:) .* e); if ~isfinite(val), val = NaN; end
end

function v = safe_scalar(x, cap), if ~isfinite(x) || abs(x) > cap, v = NaN; else v = x; end end

function y = simulate_closedloop_2dof_euler_attacked(plant_ss, sensor_ss, C_r, C_y, t, r, attack_cfg)
    % copy of local function from phase5_full_comparison
    plant_ss = ss(plant_ss); sensor_ss = ss(sensor_ss);
    A = plant_ss.A; B = plant_ss.B; C = plant_ss.C; D = plant_ss.D;
    As = sensor_ss.A; Bs = sensor_ss.B; Cs = sensor_ss.C; Ds = sensor_ss.D;
    Cr_ss = safe_controller_ss(C_r, plant_ss); Cy_ss = safe_controller_ss(C_y, plant_ss);
    Ar = Cr_ss.A; Br = Cr_ss.B; Crm = Cr_ss.C; Dr = Cr_ss.D;
    Ay = Cy_ss.A; By = Cy_ss.B; Cym = Cy_ss.C; Dy = Cy_ss.D;
    nx_p = size(A,1); nx_s = size(As,1); nx_r = size(Ar,1); nx_y = size(Ay,1);
    xp = zeros(nx_p,1); xs = zeros(nx_s,1); xr = zeros(nx_r,1); xy = zeros(nx_y,1);
    N = length(t); y = zeros(N,1); y_meas_hist = zeros(N,1); u_prev = 0;
    for k = 1:N
        if k == 1, dt = t(1); else dt = t(k)-t(k-1); end
        yk = C * xp + D * u_prev;
        if nx_s > 0, y_s = Cs * xs + Ds * yk; xs = xs + (As * xs + Bs * yk) * dt; else y_s = yk; end
        y_meas = apply_attack_scalar(y_s, t(k), attack_cfg);
        ur = Crm * xr + Dr * r(k); uy = Cym * xy + Dy * y_meas; uk = ur - uy;
        xr = xr + (Ar * xr + Br * r(k)) * dt; xy = xy + (Ay * xy + By * y_meas) * dt; xp = xp + (A * xp + B * uk) * dt;
        y(k) = C * xp + D * uk; y_meas_hist(k) = y_meas; u_prev = uk;
    end
end

function y = simulate_closedloop_pid_euler_attacked(plant_ss, sensor_ss, C_pid, t, r, attack_cfg)
    plant_ss = ss(plant_ss); sensor_ss = ss(sensor_ss);
    A = plant_ss.A; B = plant_ss.B; C = plant_ss.C; D = plant_ss.D;
    As = sensor_ss.A; Bs = sensor_ss.B; Cs = sensor_ss.C; Ds = sensor_ss.D;
    Cc_ss = safe_controller_ss(C_pid, plant_ss);
    Ac = Cc_ss.A; Bc = Cc_ss.B; Cc = Cc_ss.C; Dc = Cc_ss.D;
    nx_p = size(A,1); nx_s = size(As,1); nx_c = size(Ac,1);
    xp = zeros(nx_p,1); xs = zeros(nx_s,1); xc = zeros(nx_c,1);
    N = length(t); y = zeros(N,1); u_prev = 0;
    for k = 1:N
        if k == 1, dt = t(1); else dt = t(k)-t(k-1); end
        yk = C * xp + D * u_prev;
        if nx_s > 0, y_s = Cs * xs + Ds * yk; xs = xs + (As * xs + Bs * yk) * dt; else y_s = yk; end
        y_meas = apply_attack_scalar(y_s, t(k), attack_cfg);
        e = r(k) - y_meas; u_unclamped = Cc * xc + Dc * e; umax = 10; umin = -10; uk = min(max(u_unclamped, umin), umax);
        if abs(uk - u_unclamped) < 1e-9, xc = xc + (Ac * xc + Bc * e) * dt; else xc = xc + 0.1 * (Ac * xc + Bc * e) * dt; end
        xp = xp + (A * xp + B * uk) * dt; y(k) = C * xp + D * uk; u_prev = uk;
    end
end

function [u, mode_history, switch_times, y] = simulate_resilient_closedloop_euler(plant_ss, sensor_ss, C_r, C_y, C_pid, t, r, attack_cfg, attack_flag, detection_time, switcher_cfg)
    % Copied and slightly adapted from phase5_full_comparison
    plant_ss = ss(plant_ss); sensor_ss = ss(sensor_ss);
    A = plant_ss.A; B = plant_ss.B; C = plant_ss.C; D = plant_ss.D;
    As = sensor_ss.A; Bs = sensor_ss.B; Cs = sensor_ss.C; Ds = sensor_ss.D;
    Cr_ss = safe_controller_ss(C_r, plant_ss); Cy_ss = safe_controller_ss(C_y, plant_ss); Cpid_ss = safe_controller_ss(C_pid, plant_ss);
    Ar = Cr_ss.A; Br = Cr_ss.B; Crm = Cr_ss.C; Dr = Cr_ss.D; Ay = Cy_ss.A; By = Cy_ss.B; Cym = Cy_ss.C; Dy = Cy_ss.D;
    Ap = Cpid_ss.A; Bp = Cpid_ss.B; Cpm = Cpid_ss.C; Dp = Cpid_ss.D;
    nx_p = size(A,1); nx_s = size(As,1); nx_r = size(Ar,1); nx_y = size(Ay,1); nx_pidx = size(Ap,1);
    xp = zeros(nx_p,1); xs = zeros(nx_s,1); xr = zeros(nx_r,1); xy = zeros(nx_y,1); xpid = zeros(nx_pidx,1);
    N = length(t); y = zeros(N,1); u = zeros(N,1); mode_history = ones(N,1); switch_times = [];
    mode = 1;
    if attack_flag && isfinite(detection_time), switch_index = find(t >= detection_time, 1, 'first'); if isempty(switch_index), switch_index = N+1; end; else switch_index = N+1; end
    if ~exist('switcher_cfg','var') || isempty(switcher_cfg) || ~isfield(switcher_cfg,'blend_time'), blend_time = 0.5; else blend_time = switcher_cfg.blend_time; end
    if isfinite(detection_time), blend_end_time = detection_time + max(0, blend_time); blend_end_index = find(t >= blend_end_time, 1, 'first'); if isempty(blend_end_index), blend_end_index = switch_index; end; else blend_end_time = NaN; blend_end_index = N+1; end
    if ~exist('switcher_cfg','var') || isempty(switcher_cfg) || ~isfield(switcher_cfg,'recovery_time'), recovery_time = 1.0; else recovery_time = switcher_cfg.recovery_time; end
    if ~exist('switcher_cfg','var') || isempty(switcher_cfg) || ~isfield(switcher_cfg,'actuator_limits'), umax = 10; umin = -10; else lim = switcher_cfg.actuator_limits; if numel(lim)==2, umin = lim(1); umax = lim(2); else umax=10; umin=-10; end; end
    for k = 1:N
        if k == 1, dt = t(1); else dt = t(k)-t(k-1); end
        u_prev = 0; if k>1, u_prev = u(k-1); end
        yk = C * xp + D * u_prev;
        if nx_s > 0, y_s = Cs * xs + Ds * yk; xs = xs + (As * xs + Bs * yk) * dt; else y_s = yk; end
        y_meas = apply_attack_scalar(y_s, t(k), attack_cfg);
        if k < switch_index || ~isfinite(detection_time), y_ctrl = y_meas; elseif t(k) <= detection_time + recovery_time, beta = (t(k)-detection_time)/max(eps,recovery_time); beta=min(max(beta,0),1); y_ctrl = (1-beta)*y_meas + beta*yk; else y_ctrl = yk; end
        if k >= switch_index
            if mode ~= 2
                switch_times(end+1,:) = [t(k), mode, 2]; mode = 2;
                if nx_pidx > 0
                    epid_now = r(k) - y_ctrl;
                    if isfield(switcher_cfg,'bumpless_reg'), regval = switcher_cfg.bumpless_reg; else regval = []; end
                    xpid = align_controller_state(Cpid_ss, epid_now, u_prev, xpid, regval);
                    try pid_out_now = Cpm * xpid + Dp * epid_now; if ~isfinite(pid_out_now), pid_out_now=0; end; if abs(pid_out_now - u_prev) > 0.5 * max(1, abs(u_prev)), xpid = 0.1 * xpid; end; catch; end
                end
            end
        end
        mode_history(k) = mode;
        ur = Crm * xr + Dr * r(k); uy = Cym * xy + Dy * y_ctrl; epid = r(k) - y_ctrl; pid_out = Cpm * xpid + Dp * epid;
        if k < switch_index, alpha = 0; elseif k >= switch_index && k < blend_end_index, if isnan(blend_time) || blend_time <= 0, alpha = 1; else alpha = (t(k)-detection_time)/max(eps,blend_time); alpha = min(max(alpha,0),1); end; else alpha = 1; end
        uk_unclamped = (1 - alpha) * (ur - uy) + alpha * pid_out;
        uk = min(max(uk_unclamped, umin), umax);
        if ~isempty(Ar), xr = xr + (Ar * xr + Br * r(k)) * dt; end
        if ~isempty(Ay), xy = xy + (Ay * xy + By * y_ctrl) * dt; end
        if ~isempty(Ap)
            if abs(uk - uk_unclamped) < 1e-9, xpid = xpid + (Ap * xpid + Bp * epid) * dt; else xpid = xpid + 0.1 * (Ap * xpid + Bp * epid) * dt; end
        end
        xp = xp + (A * xp + B * uk) * dt; y(k) = C * xp + D * uk; u(k) = uk;
    end
    if isempty(switch_times), switch_times = zeros(0,3); end
end

function y_attack = apply_attack_scalar(y, t, attack_cfg)
    y_attack = y; if ~isfield(attack_cfg,'enabled') || ~attack_cfg.enabled, return; end
    if ~isfield(attack_cfg,'start_time'), attack_cfg.start_time = 0; end
    if t < attack_cfg.start_time, return; end
    switch lower(string(attack_cfg.type))
        case "bias", if isfield(attack_cfg,'magnitude'), y_attack = y + attack_cfg.magnitude; end
        case "ramp", if isfield(attack_cfg,'slope'), y_attack = y + attack_cfg.slope * (t - attack_cfg.start_time); end
        case "sine", amp = 0; freq = 1; if isfield(attack_cfg,'magnitude'), amp = attack_cfg.magnitude; end; if isfield(attack_cfg,'frequency'), freq = attack_cfg.frequency; end; y_attack = y + amp * sin(2*pi*freq*(t - attack_cfg.start_time));
        otherwise, y_attack = y;
    end
end

function ss_sys = safe_controller_ss(C, plant_ss)
    try ss_sys = ss(C); return; catch; try pid_fb = pidtune(plant_ss,'PID'); ss_sys = ss(pid_fb); return; catch; ss_sys = ss(1); return; end end
end

function x = align_controller_state(ctrl_ss, input_value, desired_output, fallback_state, reg)
    x = fallback_state;
    try
        Cc = ctrl_ss.C; Dc = ctrl_ss.D; if isempty(Cc), return; end
        target = desired_output - Dc * input_value; if isempty(ctrl_ss.A), x = zeros(size(fallback_state)); return; end
        if nargin < 5 || isempty(reg), reg = 1e-6; end
        if size(Cc,1) == 1, denom = Cc * Cc.' + reg; x = (Cc.' / denom) * target; else Regl = reg * eye(size(Cc,2)); x = (Cc.' * Cc + Regl) \ (Cc.' * target); end
        if any(~isfinite(x)), x = fallback_state; end
    catch, x = fallback_state; end
end

function [attack_flag, confidence, detection_time, residuals] = direct_baseline_detector(y_meas, y_nominal, t, detector_cfg)
    y_meas = y_meas(:); y_nominal = y_nominal(:); t = t(:);
    residuals = y_meas - y_nominal; residuals = max(min(residuals, 1e6), -1e6);
    if ~isfield(detector_cfg,'baseline_window'), detector_cfg.baseline_window = 5; end
    if ~isfield(detector_cfg,'window_size'), detector_cfg.window_size = 40; end
    if ~isfield(detector_cfg,'threshold_factor'), detector_cfg.threshold_factor = 2; end
    if ~isfield(detector_cfg,'min_consecutive'), detector_cfg.min_consecutive = 1; end
    if ~isfield(detector_cfg,'startup_suppress'), detector_cfg.startup_suppress = max(0, detector_cfg.baseline_window - 0.5); end
    sigma = std(residuals(1:min(length(t), round(detector_cfg.baseline_window / max(eps, t(2)-t(1)))))); if ~isfinite(sigma) || sigma<=0, sigma = 1e-6; end
    threshold = detector_cfg.threshold_factor * sigma; attack_flag=false; confidence=0; detection_time=NaN; exceed_count=0;
    for k=1:length(t)
        win_start = max(1, k - detector_cfg.window_size + 1); window_abs = abs(residuals(win_start:k)); Jk = mean(window_abs) + std(window_abs) + abs(residuals(k));
        if t(k) > detector_cfg.startup_suppress
            if Jk > threshold, exceed_count = exceed_count + 1; else exceed_count = 0; end
            if exceed_count >= detector_cfg.min_consecutive, attack_flag = true; confidence = min(Jk, 10); detection_time = t(k); break; end
        end
    end
end
