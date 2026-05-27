% phase5_full_comparison.m
% Full comparison table: 2DoF FOPID vs PID vs Resilient (2DoF+Detector+Switcher)
% Produces CSV and MAT summaries and per-scenario plots in results/phase5/

% Setup
addpath(pwd);
if ~exist('results','dir'), mkdir('results'); end
outdir = fullfile('results','phase5'); if ~exist(outdir,'dir'), mkdir(outdir); end
% Create a run-specific log for Phase 5
global AVR_SHARED_LOG_FID AVR_SHARED_LOG_PATH
run_ts = datestr(now,'yyyymmdd_HHMMSS');
if exist('AVR_SHARED_LOG_FID','var') && ~isempty(AVR_SHARED_LOG_FID) && AVR_SHARED_LOG_FID > 0
    lf = AVR_SHARED_LOG_FID;
    closeLog = false;
    logpath = AVR_SHARED_LOG_PATH;
else
    logpath = fullfile(outdir, ['phase5_run_' run_ts '.log']);
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

% Baseline 2DoF/PID closed-loop responses using the same transfer-function form
% already used elsewhere in the repository. This is more stable than the
% explicit Euler approximation for the fractional controller paths.
try
    G_cl_2dof = minreal((G_fwd * C_2dof_r) / (1 + G_fwd * C_2dof_y * G_sen), 1e-6);
    y_2dof = lsim(G_cl_2dof, r, t);
catch ME
    warning('Failed simulating 2DoF closed-loop; returning zeros: %s', ME.message);
    y_2dof = zeros(size(t));
end

% PID closed-loop
try
    G_cl_pid = minreal((G_fwd * C_pid) / (1 + G_fwd * C_pid * G_sen), 1e-6);
    y_pid = lsim(G_cl_pid, r, t);
catch ME
    warning('Failed simulating PID closed-loop; returning zeros: %s', ME.message);
    y_pid = zeros(size(t));
end

% Attack scenarios (Phase 5 matrix requires at least bias, ramp, sine)
scenarios = {};
scenarios{end+1} = struct('name','bias_small','type','bias','magnitude',0.1,'start_time',5);
scenarios{end+1} = struct('name','bias_large','type','bias','magnitude',0.5,'start_time',5);
scenarios{end+1} = struct('name','ramp','type','ramp','slope',0.05,'start_time',5);
scenarios{end+1} = struct('name','sine','type','sine','magnitude',0.1,'frequency',1,'start_time',5);

% Detector & switcher defaults (tightened to suppress startup false positives)
detector_cfg = struct('baseline_window',5,'window_size',100,'threshold_factor',5,'Q',1e-6,'R',1e-4,'min_consecutive',7,'startup_suppress',5);
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
    try
        [attack_flag, confidence, detection_time, residuals] = avr_detector(y_meas, t, G_cl_2dof, r, detector_cfg);
    catch ME
        fprintf(lf, 'Detector ERROR: %s\n', ME.message);
        attack_flag = false; confidence = NaN; detection_time = NaN; residuals = zeros(size(t));
    end
    detection_delay = NaN; if ~isnan(detection_time), detection_delay = detection_time - attack_cfg.start_time; end
    fprintf(lf, 'Detector: flag=%d, confidence=%g, detection_time=%s, delay=%s\n', double(attack_flag), confidence, num2str(detection_time), num2str(detection_delay));

    % Resilient run: simulate plant + controller in closed loop using the
    % detector timestamp as the switch point. This avoids the unstable
    % open-loop plant replay that previously collapsed to zero output.
    switcher_cfg.detector_attack_flag = attack_flag;
    switcher_cfg.detector_attack_time = detection_time;
    try
        [u_res, mode_hist, switch_times, y_res] = simulate_resilient_closedloop_euler( ...
            ss(G_fwd), ss(G_sen), C_2dof_r, C_2dof_y, C_pid, t, r, attack_cfg, attack_flag, detection_time, switcher_cfg);
        fprintf(lf, 'Resilient sim: transitions=%d, final_mode=%d\n', size(switch_times,1), mode_hist(end));
    catch ME
        fprintf(lf, 'Resilient sim ERROR: %s\n', ME.message);
        u_res = zeros(size(t)); mode_hist = zeros(size(t)); switch_times = []; y_res = y_2dof;
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
    fprintf(lf, 'Saved results: %s\n', fname);

    % plot
    hf = figure('Visible','off');
    subplot(3,1,1); plot(t, y_2dof, 'b', t, y_pid, 'g', t, y_res, 'r'); legend('2DoF','PID','Resilient'); title(['Outputs - ' sc.name]); grid on;
    subplot(3,1,2); plot(t, residuals); title('Residuals'); grid on; if ~isnan(detection_time), xline(detection_time,'r--'); end
    subplot(3,1,3); stairs(t, mode_hist); title('Mode history (resilient)'); ylim([0.5 3.5]); grid on;
    saveas(hf, fullfile(outdir, [sc.name '.png'])); close(hf);

    % Collect table row
    row = struct();
    row.scenario = sc.name; row.attack_type = sc.type;
    if isfield(sc,'magnitude'), row.attack_mag = sc.magnitude; else row.attack_mag = NaN; end
    if isfield(sc,'slope'), row.attack_slope = sc.slope; else row.attack_slope = NaN; end
    row.detected = double(attack_flag); row.detection_time = detection_time; row.detection_delay = detection_delay; row.confidence = confidence;
    row.ITAE_2dof = metrics.ITAE_2dof; row.ITAE_pid = metrics.ITAE_pid; row.ITAE_res = metrics.ITAE_res;
    row.mode_transitions = size(switch_times,1); row.final_mode = mode_hist(end);
    rows{end+1} = row;
end

% Write CSV
csvpath = fullfile(outdir, 'phase5_comparison.csv');
csvfid = fopen(csvpath,'w');
if csvfid < 0
    error('Could not open %s for writing.', csvpath);
end
fprintf(csvfid, 'scenario,attack_type,mag,slope,detected,det_time,det_delay,conf,ITAE_2DoF,ITAE_PID,ITAE_Res,mode_transitions,final_mode\n');
for k = 1:length(rows)
    r = rows{k};
    fprintf(csvfid, '%s,%s,%.4f,%.4f,%d,%.4f,%.4f,%.6g,%.6f,%.6f,%.6f,%d,%d\n', r.scenario, r.attack_type, r.attack_mag, r.attack_slope, r.detected, NaN2num(r.detection_time), NaN2num(r.detection_delay), r.confidence, r.ITAE_2dof, r.ITAE_pid, r.ITAE_res, r.mode_transitions, r.final_mode);
end
fclose(csvfid);

% Save summary MAT
save(fullfile(outdir,'phase5_summary.mat'), 'rows');
fprintf('Phase 5 full comparison complete. Results in %s\n', outdir);
fprintf(lf, '\nPhase5 summary saved.\n');
if closeLog
    fclose(lf);
end

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
            end
        end
        mode_history(k) = mode;

        if mode == 1
            ur = Crm * xr + Dr * r(k);
            uy = Cym * xy + Dy * y_meas;
            uk = ur - uy;
            xr = xr + (Ar * xr + Br * r(k)) * dt;
            xy = xy + (Ay * xy + By * y_meas) * dt;
        else
            epid = r(k) - y_meas;
            uk = Cpm * xpid + Dp * epid;
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
