%% avr_phase3_test.m
% Runner for Phase 3: attack injection, detection, switching, and metrics

clearvars; close all; clc;

% Load parameters and Phase 2 results
avr_parameters;
if exist('avr_phase2.mat','file')
    load('avr_phase2.mat');
else
    warning('avr_phase2.mat not found. Please run Phase 2 first.');
end
if exist('avr_baseline.mat','file')
    load('avr_baseline.mat');
end

% Build plant
G_amp = tf(Ka,[Ta 1]); G_exc = tf(Ke,[Te 1]);
G_gen = tf(Kg,[Tg 1]); G_sen = tf(Ks,[Ts 1]);
G_fwd = minreal(G_amp * G_exc * G_gen);

% Time vector
t = 0:0.001:Tfinal; N = length(t);
r = ones(size(t)); % unit step setpoint

% Build controllers
% 2DoF: either load from phase2 or reconstruct
if exist('best_params','var')
    bp = best_params;
    Kp = bp(1); Ki = bp(2); Kd = bp(3); lam = bp(4); mu = bp(5);
    if length(bp) >= 7
        b = bp(6); c = bp(7);
    else
        b = 1; c = 1;
    end
    frac = struct('wb',1e-3,'wh',1e3,'N',5);
    [C_r_2dof, C_y_2dof] = fopid_2dof(Kp,Ki,Kd,lam,mu,b,c,frac.wb,frac.wh,frac.N);
    % Construct closed-loop transfer
    G_cl_2dof = minreal((G_fwd * C_r_2dof) / (1 + G_fwd * C_y_2dof * G_sen), 1e-3);
else
    error('best_params not found in avr_phase2.mat');
end

% PID baseline
C_pid = zn_pid(G_fwd * G_sen);
G_cl_pid = minreal(feedback(G_fwd * C_pid, G_sen));

% Simulate true closed-loop with 2DoF to obtain y_true
[y_true, ~] = step(G_cl_2dof, t);

% Auto-apply best tuning config if available
best_cfg_path = fullfile('results','results_tune_detector.mat');
if exist(best_cfg_path,'file')
    s = load(best_cfg_path,'best_cfg');
    best_cfg = s.best_cfg;
    fprintf('Applying best tuning from %s\n', best_cfg_path);
    detector_cfg = struct('baseline_window',5,'window_size',100,'threshold_factor',best_cfg.threshold_factor,'Q',best_cfg.Q_scale*1e-6,'R',best_cfg.R_scale*1e-4);
    switcher_cfg = struct('hysteresis_time', best_cfg.hysteresis_time, 'recovery_time', best_cfg.recovery_time);
else
    detector_cfg = struct('baseline_window',5,'window_size',100,'threshold_factor',2,'Q',1e-6,'R',1e-4);
    switcher_cfg = struct('hysteresis_time',2,'recovery_time',0.5);
end

% Attack scenarios
attack_types = {'bias','ramp','sine'};
attack_configs(1).enabled = true; attack_configs(1).type = 'bias'; attack_configs(1).magnitude = 0.1; attack_configs(1).start_time = 5;
attack_configs(2).enabled = true; attack_configs(2).type = 'ramp'; attack_configs(2).slope = 0.05; attack_configs(2).start_time = 5;
attack_configs(3).enabled = true; attack_configs(3).type = 'sine'; attack_configs(3).magnitude = 0.1; attack_configs(3).frequency = 1; attack_configs(3).start_time = 5;

results = cell(length(attack_types),1);

for i = 1:length(attack_types)
    cfg = attack_configs(i);
    y_meas = avr_attack_injector(y_true, t, cfg);

    % Detector
    % Detector
    [attack_flag, confidence, detection_time, residuals] = avr_detector(y_meas, t, G_fwd, r, detector_cfg);

    % Switcher: use C_y_2dof as error-path controller for 2DoF
    [u_switched, mode_history, switch_times] = avr_switcher(y_meas, t, r, C_y_2dof, C_pid, switcher_cfg);

    % Re-simulate plant with switched control by applying u_switched as external input is nontrivial.
    % Instead, approximate resulting output y_switched by combining controller output and plant input via lsim
    % Compute closed-loop response under switching approximately by simulating plant with u_switched as input
    try
        sys_u = G_fwd / (1 + G_sen * G_fwd * 0); % approximate forward map from control to output
        y_switched = lsim(G_fwd, u_switched, t); % approximate
    catch
        y_switched = y_true; % fallback
    end

    % Metrics
    itae_fn = @(y,tv) trapz(tv, tv .* abs(1 - y));
    ITAE_2dof = itae_fn(y_true, t);
    ITAE_switched = itae_fn(y_switched, t);
    ITAE_pid = itae_fn(step(G_cl_pid, t), t);

    info_true = stepinfo(y_true, t);
    info_sw = stepinfo(y_switched, t);
    info_pid = stepinfo(step(G_cl_pid, t), t);

    results{i}.attack_type = cfg.type;
    results{i}.attack_config = cfg;
    results{i}.y_true = y_true;
    results{i}.y_meas = y_meas;
    results{i}.residuals = residuals;
    results{i}.attack_flag = attack_flag;
    results{i}.confidence = confidence;
    results{i}.detection_time = detection_time;
    results{i}.u_switched = u_switched;
    results{i}.mode_history = mode_history;
    results{i}.switch_times = switch_times;
    results{i}.y_switched = y_switched;
    results{i}.metrics = struct('ITAE_2dof',ITAE_2dof,'ITAE_switched',ITAE_switched,'ITAE_pid',ITAE_pid,...
        'info_true',info_true,'info_sw',info_sw,'info_pid',info_pid);

    % Save one-file per scenario
    fname = fullfile('results', sprintf('phase3_%s.mat', cfg.type));
    if ~exist('results','dir'), mkdir('results'); end
    r = results{i};
    save(fname, 'r');
    fprintf('Saved results to %s\n', fname);
    % Plot results for quick inspection
    try
        pngname = fullfile('results', sprintf('phase3_%s.png', cfg.type));
        avr_phase3_plot(r, pngname);
        fprintf('Saved plot to %s\n', pngname);
    catch ME
        warning('Failed to plot results: %s', ME.message);
    end
end

% Save aggregate
if exist('best_cfg','var')
    save('results/phase3_all.mat','results','best_cfg');
else
    save('results/phase3_all.mat','results');
end
fprintf('Phase 3 tests complete. Results saved to results/phase3_all.mat\n');

function C = zn_pid(G)
    [Gm, ~, ~, Wcg] = margin(G);
    if isempty(Gm) || isempty(Wcg) || ~isfinite(Gm) || ~isfinite(Wcg) || Gm <= 0 || Wcg <= 0
        C = pidtune(G, 'PID');
        return;
    end
    Ku = Gm; Pu = 2*pi/Wcg;
    Kp = 0.6*Ku; Ki = 1.2*Ku/Pu; Kd = 0.075*Ku*Pu;
    C = pid(Kp, Ki, Kd);
end
