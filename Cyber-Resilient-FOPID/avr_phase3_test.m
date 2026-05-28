%% avr_phase3_test.m
% Runner for Phase 3: attack injection and detection metrics

clearvars; close all; clc;

paths3 = phase_artifacts('phase3');
outdir = paths3.root;
plotdir = paths3.plots;
matdir = paths3.mat;
csvdir = paths3.csv;

% Load parameters and Phase 2 results
avr_parameters;
phase2mat = fullfile(phase_artifacts('phase2').mat, 'avr_phase2.mat');
if exist(phase2mat,'file')
    load(phase2mat);
else
    warning('avr_phase2.mat not found. Please run Phase 2 first.');
end
phase1mat = fullfile(phase_artifacts('phase1').mat, 'avr_baseline.mat');
if exist(phase1mat,'file')
    load(phase1mat);
end

% Build plant
G_amp = tf(Ka,[Ta 1]); G_exc = tf(Ke,[Te 1]);
G_gen = tf(Kg,[Tg 1]); G_sen = tf(Ks,[Ts 1]);
G_fwd = minreal(G_amp * G_exc * G_gen);

% Time vector
t = 0:0.001:Tfinal; N = length(t);
r = ones(size(t)); % unit step setpoint

% Build 2DoF baseline controller for detector benchmarking
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
    G_cl_2dof = minreal((G_fwd * C_r_2dof) / (1 + G_fwd * C_y_2dof * G_sen), 1e-3);
else
    error('best_params not found in avr_phase2.mat');
end

% Simulate true closed-loop with 2DoF to obtain y_true
[y_true, ~] = step(G_cl_2dof, t);

% Auto-apply best tuning config if available
paths3 = phase_artifacts('phase3');
best_cfg_path = fullfile(paths3.mat,'results_tune_detector.mat');
if exist(best_cfg_path,'file')
    s = load(best_cfg_path,'best_cfg');
    best_cfg = s.best_cfg;
    fprintf('Applying best tuning from %s\n', best_cfg_path);
    detector_cfg = struct('baseline_window',5,'window_size',100,'threshold_factor',best_cfg.threshold_factor,'Q',best_cfg.Q_scale*1e-6,'R',best_cfg.R_scale*1e-4);
    %#ok<NASGU>
else
    detector_cfg = struct('baseline_window',6,'window_size',200,'threshold_factor',5,'Q',1e-6,'R',1e-4,'min_consecutive',7,'startup_suppress',6);
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
    [attack_flag, confidence, detection_time, residuals] = avr_detector(y_meas, t, G_cl_2dof, r, detector_cfg);

    residual_rms = sqrt(mean(residuals.^2));
    residual_peak = max(abs(residuals));

    results{i}.attack_type = cfg.type;
    results{i}.attack_config = cfg;
    results{i}.y_true = y_true;
    results{i}.y_meas = y_meas;
    results{i}.residuals = residuals;
    results{i}.attack_flag = attack_flag;
    results{i}.confidence = confidence;
    results{i}.detection_time = detection_time;
    results{i}.residual_rms = residual_rms;
    results{i}.residual_peak = residual_peak;
    results{i}.metrics = struct('residual_rms',residual_rms,'residual_peak',residual_peak);

    % Save one-file per scenario
    fname = fullfile(matdir, sprintf('phase3_%s.mat', cfg.type));
    r = results{i};
    save(fname, 'r');
    fprintf('Saved results to %s\n', fname);
    % Plot results for quick inspection
    try
        pngname = fullfile(plotdir, sprintf('phase3_%s.png', cfg.type));
        avr_phase3_plot(r, pngname);
        fprintf('Saved plot to %s\n', pngname);
    catch ME
        warning('Failed to plot results: %s', ME.message);
    end
end

% Save aggregate
if exist('best_cfg','var')
    save(fullfile(matdir,'phase3_all.mat'),'results','best_cfg');
else
    save(fullfile(matdir,'phase3_all.mat'),'results');
end
fprintf('Phase 3 tests complete. Results saved to %s\n', fullfile(matdir,'phase3_all.mat'));

