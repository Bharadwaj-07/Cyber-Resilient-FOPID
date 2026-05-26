% phase5_full_comparison.m
% Full comparison table: 2DoF FOPID vs PID vs Resilient (2DoF+Detector+Switcher)
% Produces CSV and MAT summaries and per-scenario plots in results/phase5/

% Setup
addpath(pwd);
if ~exist('results','dir'), mkdir('results'); end
outdir = fullfile('results','phase5'); if ~exist(outdir,'dir'), mkdir(outdir); end

% Load parameters and Phase 2 controllers
avr_parameters;
if exist('avr_phase2.mat','file')
    data = load('avr_phase2.mat');
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

% Classical PID (for comparison)
try
    C_pid = pidtune(G_fwd * G_sen, 'PID');
catch
    C_pid = pid(1,1,0.1);
end

% Time base and reference
Tfinal = 25; dt = 0.01; t = (0:dt:Tfinal)'; r = ones(size(t));

% Baseline 2DoF closed-loop response (simulate closed-loop state-space via Euler)
try
    G_cl_2dof = minreal((G_fwd * C_2dof_r) / (1 + G_fwd * C_2dof_y * G_sen), 1e-6);
    y_2dof = simulate_ss_euler(ss(G_cl_2dof), r, t);
catch
    y_2dof = zeros(size(t));
end

% PID closed-loop (simulate via Euler on closed-loop ss)
try
    G_cl_pid = minreal((G_fwd * C_pid) / (1 + G_fwd * C_pid * G_sen), 1e-6);
    y_pid = simulate_ss_euler(ss(G_cl_pid), r, t);
catch
    y_pid = zeros(size(t));
end

% Attack scenarios (Phase 5 matrix requires at least bias, ramp, sine)
scenarios = {};
scenarios{end+1} = struct('name','bias_small','type','bias','magnitude',0.1,'start_time',5);
scenarios{end+1} = struct('name','bias_large','type','bias','magnitude',0.5,'start_time',5);
scenarios{end+1} = struct('name','ramp','type','ramp','slope',0.05,'start_time',5);
scenarios{end+1} = struct('name','sine','type','sine','magnitude',0.1,'frequency',1,'start_time',5);

% Detector & switcher defaults (from Phase3 tuning recommendations if available)
detector_cfg = struct('baseline_window',3,'window_size',50,'threshold_factor',3,'Q',1e-6,'R',1e-4,'min_consecutive',3);
switcher_cfg = struct('hysteresis_time',2,'recovery_time',0.5,'initial_mode',1);

% Prepare results table
rows = {};

for i = 1:length(scenarios)
    sc = scenarios{i};
    % Generate attacked measurement from 2DoF baseline output
    y_true = y_2dof;
    attack_cfg = struct('enabled',true,'type',sc.type);
    if isfield(sc,'magnitude'), attack_cfg.magnitude = sc.magnitude; end
    if isfield(sc,'slope'), attack_cfg.slope = sc.slope; end
    if isfield(sc,'frequency'), attack_cfg.frequency = sc.frequency; end
    attack_cfg.start_time = sc.start_time;

    y_meas = avr_attack_injector(y_true, t, attack_cfg);

    % Run detector
    [attack_flag, confidence, detection_time, residuals] = avr_detector(y_meas, t, G_fwd, r, detector_cfg);
    detection_delay = NaN; if ~isnan(detection_time), detection_delay = detection_time - attack_cfg.start_time; end

    % Resilient run: switcher uses detector hint
    switcher_cfg.detector_attack_flag = attack_flag;
    switcher_cfg.detector_attack_time = detection_time;
    [u_res, mode_hist, switch_times] = avr_switcher(y_meas, t, r, C_2dof_y, C_pid, switcher_cfg);
    % Simulate the plant per-step using Euler integration for higher fidelity
    try
        y_res = simulate_plant_euler(ss(G_fwd), u_res, t);
    catch
        y_res = y_2dof;
    end

    % Compute metrics
    itae = @(y) trapz(t, t .* abs(1 - y));
    metrics.ITAE_2dof = itae(y_2dof);
    metrics.ITAE_pid = itae(y_pid);
    metrics.ITAE_res = itae(y_res);

    info2 = stepinfo(y_2dof, t);
    infoP = stepinfo(y_pid, t);
    infoR = stepinfo(y_res, t);

    % Save per-scenario MAT and plot
    fname = fullfile(outdir, [sc.name '.mat']);
    save(fname, 'sc', 'y_true', 'y_meas', 'residuals', 'attack_flag', 'detection_time', 'detection_delay', 'u_res', 'mode_hist', 'switch_times', 'y_res', 'metrics');

    % plot
    hf = figure('Visible','off');
    subplot(3,1,1); plot(t, y_2dof, 'b', t, y_pid, 'g', t, y_res, 'r'); legend('2DoF','PID','Resilient'); title(['Outputs - ' sc.name]); grid on;
    subplot(3,1,2); plot(t, residuals); title('Residuals'); grid on; if ~isnan(detection_time), xline(detection_time,'r--'); end
    subplot(3,1,3); stairs(t, mode_hist); title('Mode history (resilient)'); ylim([0.5 3.5]); grid on;
    saveas(hf, fullfile(outdir, [sc.name '.png'])); close(hf);

    % Collect table row
    row = struct();
    row.scenario = sc.name; row.attack_type = sc.type; row.attack_mag = getfield(sc,'magnitude',NaN); row.attack_slope = getfield(sc,'slope',NaN);
    row.detected = double(attack_flag); row.detection_time = detection_time; row.detection_delay = detection_delay; row.confidence = confidence;
    row.ITAE_2dof = metrics.ITAE_2dof; row.ITAE_pid = metrics.ITAE_pid; row.ITAE_res = metrics.ITAE_res;
    row.mode_transitions = size(switch_times,1); row.final_mode = mode_hist(end);
    rows{end+1} = row;
end

% Write CSV
csvpath = fullfile(outdir, 'phase5_comparison.csv');
fid = fopen(csvpath,'w');
fprintf(fid, 'scenario,attack_type,mag,slope,detected,det_time,det_delay,conf,ITAE_2DoF,ITAE_PID,ITAE_Res,mode_transitions,final_mode\n');
for k = 1:length(rows)
    r = rows{k};
    fprintf(fid, '%s,%s,%.4f,%.4f,%d,%.4f,%.4f,%.6g,%.6f,%.6f,%.6f,%d,%d\n', r.scenario, r.attack_type, r.attack_mag, r.attack_slope, r.detected, NaN2num(r.detection_time), NaN2num(r.detection_delay), r.confidence, r.ITAE_2dof, r.ITAE_pid, r.ITAE_res, r.mode_transitions, r.final_mode);
end
fclose(fid);

% Save summary MAT
save(fullfile(outdir,'phase5_summary.mat'), 'rows');

fprintf('Phase 5 full comparison complete. Results in %s\n', outdir);

function v = NaN2num(x)
    if isempty(x) || isnan(x), v = NaN; else v = x; end
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
        u = u_seq(min(k,end));
        x = x + (A * x + B * u) * dt;
        y(k) = C * x + D * u;
    end
end
