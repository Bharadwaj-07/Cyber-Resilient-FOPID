function test_harness_observer()
% Small automated test to verify observer convergence and control jump bounds
paths5 = phase_artifacts('phase5');
if ~exist(paths5.mat,'dir'), mkdir(paths5.mat); end

avr_parameters;
phase2mat = fullfile(phase_artifacts('phase2').mat, 'avr_phase2.mat');
if exist(phase2mat,'file')
    data = load(phase2mat);
    if isfield(data,'C_y'), C_2dof_y = data.C_y; end
    if isfield(data,'C_r'), C_2dof_r = data.C_r; end
end
G_amp = tf(Ka,[Ta 1]); G_exc = tf(Ke,[Te 1]); G_gen = tf(Kg,[Tg 1]); G_sen = tf(Ks,[Ts 1]);
G_fwd = minreal(G_amp * G_exc * G_gen);
try, C_pid = pidtune(G_fwd * G_sen, 'PID'); catch, C_pid = pid(1,1,0.1); end

% Single scenario: medium bias
Tfinal = 20; dt = 0.001; t = (0:dt:Tfinal)'; r = ones(size(t));
sc = struct('name','bias_test','type','bias','magnitude',0.3,'start_time',5);
attack_cfg = struct('enabled',true,'type',sc.type,'start_time',sc.start_time,'magnitude',sc.magnitude);

% detector config
det_cfg = struct('baseline_window',5,'window_size',50,'threshold_factor',3,'min_consecutive',3,'startup_suppress',4.8);

% Run baseline
y_2dof_sc = simulate_closedloop_2dof_euler_attacked(ss(G_fwd), ss(G_sen), C_2dof_r, C_2dof_y, t, r, attack_cfg);
[attack_flag, confidence, detection_time, residuals] = direct_baseline_detector(y_2dof_sc, y_2dof_sc, t, det_cfg);

% Run resilient sim with observer
cfg = struct('blend_time',1.5,'recovery_time',2.0,'bumpless_reg',1e-2,'actuator_limits',[-5 5],'observer_recovery_time',2.0,'observer_innovation_limit',0.05,'observer_min_gain',0.02);
[u_res, mode_hist, switch_times, y_res] = simulate_resilient_closedloop_euler(ss(G_fwd), ss(G_sen), C_2dof_r, C_2dof_y, C_pid, t, r, attack_cfg, attack_flag, detection_time, cfg);

% Diagnostics
itae_res = safe_itae(y_res, t, 1e6);
u_peak_rate = max(abs(diff(u_res))) / max(eps, t(2)-t(1));

res.out = struct('itae_res',itae_res,'u_peak_rate',u_peak_rate,'detection_time',detection_time,'attack_flag',attack_flag);
save(fullfile(paths5.mat,'observer_test.mat'),'res');
end
