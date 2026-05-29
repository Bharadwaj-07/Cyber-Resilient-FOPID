function phase5_augmented_compare()
% Compare baseline vs augmented (resilient) versions of PID, 1DoF and 2DoF.
paths5 = phase_artifacts('phase5'); outcsv = fullfile(paths5.csv,'phase5_augmented_comparison.csv');

% Load system and controllers
avr_parameters;
phase2mat = fullfile(phase_artifacts('phase2').mat, 'avr_phase2.mat');
data = struct(); if exist(phase2mat,'file'), data = load(phase2mat); end
G_amp = tf(Ka,[Ta 1]); G_exc = tf(Ke,[Te 1]); G_gen = tf(Kg,[Tg 1]); G_sen = tf(Ks,[Ts 1]);
G_fwd = minreal(G_amp * G_exc * G_gen);

% Controllers: fallbacks where missing
if isfield(data,'C_y'), C_2dof_y = data.C_y; end
if isfield(data,'C_r'), C_2dof_r = data.C_r; end
if isfield(data,'C_y_1dof'), C_1dof = data.C_y_1dof; end
if isfield(data,'C_pid'), C_pid = data.C_pid; end
if ~exist('C_pid','var') || isempty(C_pid)
    try C_pid = pidtune(G_fwd * G_sen, 'PID'); catch, C_pid = pid(1,1,0.1); end
end
if ~exist('C_1dof','var') || isempty(C_1dof)
    C_1dof = C_pid;
end
if ~exist('C_2dof_y','var') || isempty(C_2dof_y)
    try, C_2dof_y = pidtune(G_fwd * G_sen, 'PID'); C_2dof_r = C_2dof_y; catch, C_2dof_y = pid(1,1,0.1); C_2dof_r = C_2dof_y; end
end

% Scenarios
Tfinal = 25; dt = 0.002; t = (0:dt:Tfinal)'; r = ones(size(t));
scenarios = {};
scenarios{end+1} = struct('name','bias_small','type','bias','magnitude',0.1,'start_time',5);
scenarios{end+1} = struct('name','bias_large','type','bias','magnitude',0.5,'start_time',5);
scenarios{end+1} = struct('name','ramp','type','ramp','slope',0.05,'start_time',5);
scenarios{end+1} = struct('name','sine','type','sine','magnitude',0.1,'frequency',1,'start_time',5);

results = [];
row = 0;
for is = 1:numel(scenarios)
    sc = scenarios{is}; attack_cfg = struct('enabled',true,'type',sc.type,'start_time',sc.start_time);
    if isfield(sc,'magnitude'), attack_cfg.magnitude = sc.magnitude; end
    if isfield(sc,'slope'), attack_cfg.slope = sc.slope; end
    if isfield(sc,'frequency'), attack_cfg.frequency = sc.frequency; end

    % Baseline PID
    y_pid = simulate_closedloop_pid_euler_attacked(ss(G_fwd), ss(G_sen), C_pid, t, r, attack_cfg);
    y_pid = sanitize_signal(y_pid); itae_pid = safe_itae(y_pid,t,1e6);

    % Augmented PID (apply resilient wrapper): use C_r = 0, C_y = C_pid
    C_r_zero = tf(0);
    [u_res_pid,~,switch_times_pid,y_res_pid,diag_pid] = simulate_resilient_closedloop_euler(ss(G_fwd), ss(G_sen), C_r_zero, C_pid, C_pid, t, r, attack_cfg, 1, 5, struct('blend_time',0.5,'isolation_tau',0.25,'observer_recovery_time',1.0,'actuator_limits',[-5 5]));
    y_res_pid = sanitize_signal(y_res_pid); itae_res_pid = safe_itae(y_res_pid,t,1e6);

    % Baseline 1DoF (feedback-only)
    y_1dof = simulate_closedloop_pid_euler_attacked(ss(G_fwd), ss(G_sen), C_1dof, t, r, attack_cfg);
    y_1dof = sanitize_signal(y_1dof); itae_1dof = safe_itae(y_1dof,t,1e6);

    % Augmented 1DoF
    [u_res_1d,~,switch_times_1d,y_res_1d,diag_1d] = simulate_resilient_closedloop_euler(ss(G_fwd), ss(G_sen), C_r_zero, C_1dof, C_1dof, t, r, attack_cfg, 1, 5, struct('blend_time',0.5,'isolation_tau',0.25,'observer_recovery_time',1.0,'actuator_limits',[-5 5]));
    y_res_1d = sanitize_signal(y_res_1d); itae_res_1d = safe_itae(y_res_1d,t,1e6);

    % Baseline 2DoF
    [y_2dof,~] = simulate_closedloop_2dof_euler_attacked(ss(G_fwd), ss(G_sen), C_2dof_r, C_2dof_y, t, r, attack_cfg);
    y_2dof = sanitize_signal(y_2dof); itae_2dof = safe_itae(y_2dof,t,1e6);

    % Augmented 2DoF (resilient) - run as before
    [u_res_2d,~,switch_times_2d,y_res_2d,diag_2d] = simulate_resilient_closedloop_euler(ss(G_fwd), ss(G_sen), C_2dof_r, C_2dof_y, C_pid, t, r, attack_cfg, 1, 5, struct('blend_time',0.5,'isolation_tau',0.25,'observer_recovery_time',1.0,'actuator_limits',[-5 5]));
    y_res_2d = sanitize_signal(y_res_2d); itae_res_2d = safe_itae(y_res_2d,t,1e6);

    row = row + 1;
    results(row).scenario = sc.name;
    results(row).itae_pid = itae_pid; results(row).itae_aug_pid = itae_res_pid;
    results(row).itae_1d = itae_1dof; results(row).itae_aug_1d = itae_res_1d;
    results(row).itae_2d = itae_2dof; results(row).itae_aug_2d = itae_res_2d;
    results(row).u_jump_aug_pid = compute_u_jump(u_res_pid,t,switch_times_pid);
    results(row).u_jump_aug_1d = compute_u_jump(u_res_1d,t,switch_times_1d);
    results(row).u_jump_aug_2d = compute_u_jump(u_res_2d,t,switch_times_2d);
end

T = struct2table(results);
if ~exist(paths5.csv,'dir'), mkdir(paths5.csv); end
writetable(T,outcsv);
fprintf('Wrote augmented comparison to %s\n', outcsv);
end

function uj = compute_u_jump(u,t,switch_times)
    uj = NaN;
    if ~isempty(switch_times)
        idx = find(t >= switch_times(1,1),1,'first'); if isempty(idx), idx = numel(t); end
        uj = u(min(numel(u),idx)) - u(max(1,idx-1));
    end
end
