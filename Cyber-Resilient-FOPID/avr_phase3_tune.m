%% avr_phase3_tune.m
% Auto-tune detector parameters (threshold_factor, Q_scale, R_scale)
% Produces results/results_tune_detector.mat with tuning grid and chosen best config

% Preserve caller workspace state when invoked from run_all_phases.
close all; clc;

% Load parameters and Phase 2 results
avr_parameters;
phase2mat = fullfile(phase_artifacts('phase2').mat, 'avr_phase2.mat');
if exist(phase2mat,'file')
    load(phase2mat);
else
    error('avr_phase2.mat not found. Run Phase 2 first.');
end

% Build plant and 2DoF closed-loop to get y_true
G_amp = tf(Ka,[Ta 1]); G_exc = tf(Ke,[Te 1]); G_gen = tf(Kg,[Tg 1]); G_sen = tf(Ks,[Ts 1]);
G_fwd = minreal(G_amp * G_exc * G_gen);

% Build 2DoF controller from best_params
if exist('best_params','var')
    bp = best_params; Kp=bp(1); Ki=bp(2); Kd=bp(3); lam=bp(4); mu=bp(5);
    if length(bp) >= 7, b=bp(6); c=bp(7); else b=1; c=1; end
    frac = struct('wb',1e-3,'wh',1e3,'N',5);
    [C_r_2dof, C_y_2dof] = fopid_2dof(Kp,Ki,Kd,lam,mu,b,c,frac.wb,frac.wh,frac.N);
    G_cl_2dof = minreal((G_fwd * C_r_2dof) / (1 + G_fwd * C_y_2dof * G_sen), 1e-3);
else
    error('best_params not found.');
end

% Time and true response
Tfinal = 25;
t = 0:0.001:Tfinal; r = ones(size(t));
[y_true, ~] = step(G_cl_2dof, t);

% Attack scenarios: evaluate across bias, ramp, sine
attack_list = {'bias','ramp','sine'};
attack_configs = cell(3,1);
attack_configs{1} = struct('enabled',true,'type','bias','magnitude',0.1,'start_time',5);
attack_configs{2} = struct('enabled',true,'type','ramp','slope',0.05,'start_time',5);
attack_configs{3} = struct('enabled',true,'type','sine','magnitude',0.1,'frequency',1,'start_time',5);

default_detector = struct('baseline_window',5,'window_size',100,'threshold_factor',2,'Q',1e-6,'R',1e-4);

% Parameter grids (coarse by default)
threshold_factors = [1.2,1.5,1.8,2.0];
Q_scales = [0.5,1,5];
R_scales = [0.5,1,5];

% Results storage
idx = 1;
results_grid = [];

% Prepare PID for switcher simulations
C_pid = zn_pid(G_fwd * G_sen);

for tfac = threshold_factors
    for qsc = Q_scales
        for rsc = R_scales
            detector_cfg = default_detector;
            detector_cfg.threshold_factor = tfac;
            detector_cfg.Q = qsc * default_detector.Q;
            detector_cfg.R = rsc * default_detector.R;

            total_score = 0;
            attack_infos = struct();

            % Evaluate across all attack types
            for aidx = 1:length(attack_list)
                cfg = attack_configs{aidx};
                y_meas = avr_attack_injector(y_true, t, cfg);

                % Detector
                [attack_flag, confidence, detection_time, residuals] = avr_detector(y_meas, t, G_cl_2dof, r, detector_cfg);

                % Metrics
                fp = 0; if attack_flag && detection_time < cfg.start_time, fp = 1; end
                if isempty(detection_time) || isnan(detection_time)
                    dt = Inf; miss = 1; else dt = detection_time; miss = 0; end

                % Scoring: detection latency only, with heavy penalties for misses/false positives.
                score = dt + 1000*miss + 10000*fp;
                total_score = total_score + score;

                attack_infos(aidx).type = cfg.type;
                attack_infos(aidx).attack_flag = attack_flag;
                attack_infos(aidx).detection_time = dt;
                attack_infos(aidx).false_positive = fp;
                attack_infos(aidx).confidence = confidence;
            end

            results_grid(idx).threshold_factor = tfac;
            results_grid(idx).Q_scale = qsc;
            results_grid(idx).R_scale = rsc;
            results_grid(idx).total_score = total_score;
            results_grid(idx).attack_infos = attack_infos;
            idx = idx + 1;
        end
    end
end

% Choose best config: minimize total_score
best_score = Inf; best_idx = 0;
for k = 1:length(results_grid)
    if results_grid(k).total_score < best_score
        best_score = results_grid(k).total_score;
        best_idx = k;
    end
end

best_cfg = results_grid(best_idx);

% Save
paths3 = phase_artifacts('phase3');
save(fullfile(paths3.mat, 'results_tune_detector.mat'),'results_grid','best_cfg');

try
    summaryTable = table(best_cfg.threshold_factor, best_cfg.Q_scale, best_cfg.R_scale, best_cfg.total_score, ...
        'VariableNames', {'threshold_factor','Q_scale','R_scale','total_score'});
    writetable(summaryTable, fullfile(paths3.csv, 'phase3_tune_summary.csv'));
catch ME
    warning('Could not write Phase 3 tuning CSV summary: %s', ME.message);
end

% Report
fprintf('Tuning complete across %d attacks. Best config: threshold_factor=%.2f, Q_scale=%.2f, R_scale=%.2f\n', length(attack_list), best_cfg.threshold_factor, best_cfg.Q_scale, best_cfg.R_scale);
fprintf('Aggregate score = %.3f\n', best_cfg.total_score);

% Print per-attack details for the best config
fprintf('\nPer-attack detection results for best config:\n');
for aidx = 1:length(best_cfg.attack_infos)
    ai = best_cfg.attack_infos(aidx);
    fprintf('  %s: detected=%d, det_time=%.4f, false_positive=%d, conf=%.4f\n', ai.type, ai.attack_flag, ai.detection_time, ai.false_positive, ai.confidence);
end

fprintf('\nSuggested detector_config to use in Phase3: threshold_factor=%.2f, Q=%.1e, R=%.1e\n', best_cfg.threshold_factor, best_cfg.Q_scale*default_detector.Q, best_cfg.R_scale*default_detector.R);

function C = zn_pid(G)
    % Local Ziegler-Nichols-style fallback so Phase 3 does not depend on
    % helpers defined in other scripts.
    try
        [Gm, ~, ~, Wcg] = margin(G);
        if isempty(Gm) || isempty(Wcg) || ~isfinite(Gm) || ~isfinite(Wcg) || Gm <= 0 || Wcg <= 0
            C = pidtune(G, 'PID');
            return;
        end

        Ku = Gm;
        Pu = 2*pi/Wcg;
        Kp = 0.6*Ku;
        Ki = 1.2*Ku/Pu;
        Kd = 0.075*Ku*Pu;
        C = pid(Kp, Ki, Kd);
    catch
        C = pidtune(G, 'PID');
    end
end
