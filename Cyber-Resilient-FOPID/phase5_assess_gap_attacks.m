function phase5_assess_gap_attacks()
% Compare continuous attack versus repeated attack with recovery gaps.
% Variants: baseline, A attack subtraction, B anti-windup, C aggressive observer, ALL.
paths5 = phase_artifacts('phase5');
outcsv = fullfile(paths5.csv, 'phase5_assess_gap_attacks.csv');
outmat = fullfile(paths5.mat, 'phase5_assess_gap_attacks.mat');

avr_parameters;
phase2mat = fullfile(phase_artifacts('phase2').mat, 'avr_phase2.mat');
if exist(phase2mat,'file')
    data = load(phase2mat);
    if isfield(data,'C_y'), C_2dof_y = data.C_y; end
    if isfield(data,'C_r'), C_2dof_r = data.C_r; end
end
G_amp = tf(Ka,[Ta 1]); G_exc = tf(Ke,[Te 1]); G_gen = tf(Kg,[Tg 1]); G_sen = tf(Ks,[Ts 1]);
G_fwd = minreal(G_amp * G_exc * G_gen);
if ~exist('C_2dof_y','var') || isempty(C_2dof_y)
    try, C_2dof_y = pidtune(G_fwd * G_sen, 'PID'); C_2dof_r = C_2dof_y; catch, C_2dof_y = pid(1,1,0.1); C_2dof_r = C_2dof_y; end
end
try, C_pid = pidtune(G_fwd * G_sen, 'PID'); catch, C_pid = pid(1,1,0.1); end

Tfinal = 35; dt = 0.001; t = (0:dt:Tfinal)'; rref = ones(size(t));

attack_cases = {
    struct('name','continuous_bias','type','bias','magnitude',0.5,'start_time',5),
    struct('name','gapped_bias','type','bias','magnitude',0.5,'start_time',5,'burst_on_time',1.0,'burst_off_time',4.0,'burst_cycles',4)
};

variants = {
    struct('name','baseline','use_attack_subtraction',false,'anti_windup_gain',0,'use_aggressive_obs_gain',false),
    struct('name','A_attack_subtraction','use_attack_subtraction',true,'anti_windup_gain',0,'use_aggressive_obs_gain',false),
    struct('name','B_anti_windup','use_attack_subtraction',false,'anti_windup_gain',0.1,'use_aggressive_obs_gain',false),
    struct('name','C_aggr_obs_gain','use_attack_subtraction',false,'anti_windup_gain',0,'use_aggressive_obs_gain',true),
    struct('name','ALL','use_attack_subtraction',true,'anti_windup_gain',0.1,'use_aggressive_obs_gain',true)
};

results = [];
row = 0;
for v = 1:numel(variants)
    vc = variants{v};
    for a = 1:numel(attack_cases)
        ac = attack_cases{a};
        attack_cfg = ac;
        attack_cfg.enabled = true;

        switcher_cfg = struct('blend_time',0.5,'recovery_time',1.0,'isolation_tau',0.25,'Q_scale',1,'R_scale',1,'actuator_limits',[-5 5], ...
            'anti_windup_gain',vc.anti_windup_gain,'use_attack_subtraction',vc.use_attack_subtraction,'use_aggressive_obs_gain',vc.use_aggressive_obs_gain);

        y_2dof_sc = simulate_closedloop_2dof_euler_attacked(ss(G_fwd), ss(G_sen), C_2dof_r, C_2dof_y, t, rref, attack_cfg);
        [attack_flag, ~, detection_time, ~] = direct_baseline_detector(y_2dof_sc, y_2dof_sc, t, struct('baseline_window',5,'window_size',50,'threshold_factor',3,'min_consecutive',3,'startup_suppress',4.8));
        [u_res, mode_hist, switch_times, y_res, diag] = simulate_resilient_closedloop_euler(ss(G_fwd), ss(G_sen), C_2dof_r, C_2dof_y, C_pid, t, rref, attack_cfg, attack_flag, detection_time, switcher_cfg);

        y_res = sanitize_signal(y_res);
        itae_res = safe_itae(y_res, t, 1e6);
        dt_sim = t(2)-t(1);
        if ~isempty(switch_times)
            idx_sw = find(t >= switch_times(1,1), 1, 'first'); if isempty(idx_sw), idx_sw = numel(t); end
            u_jump = u_res(min(numel(u_res), idx_sw)) - u_res(max(1, idx_sw-1));
        else
            u_jump = NaN;
        end
        if numel(u_res) >= 2, u_peak_rate = max(abs(diff(u_res))) / max(eps, dt_sim); else, u_peak_rate = NaN; end

        row = row + 1;
        results(row).variant = vc.name;
        results(row).attack_case = ac.name;
        results(row).itae = itae_res;
        results(row).u_jump = u_jump;
        results(row).u_peak_rate = u_peak_rate;
        results(row).attack_detected = attack_flag;
        results(row).detection_time = detection_time;
    end
end

if ~exist(paths5.csv,'dir'), mkdir(paths5.csv); end
T = struct2table(results);
writetable(T, outcsv);
if ~exist(paths5.mat,'dir'), mkdir(paths5.mat); end
save(outmat, 'results', 'variants', 'attack_cases');
fprintf('Gap-attacks assessment complete: %s\n', outcsv);
end
