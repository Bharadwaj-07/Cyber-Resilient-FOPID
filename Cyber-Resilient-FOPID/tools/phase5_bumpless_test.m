% phase5_bumpless_test.m
% Quick local test script to compare original vs bumpless resilient behaviour
% Run this after pulling the repo changes in MATLAB.

paths5 = phase_artifacts('phase5');
% Load controllers
avr_parameters;
phase2mat = fullfile(phase_artifacts('phase2').mat, 'avr_phase2.mat');
if exist(phase2mat,'file')
    data = load(phase2mat);
    if isfield(data,'C_y'), C_2dof_y = data.C_y; end
    if isfield(data,'C_r'), C_2dof_r = data.C_r; end
end
G_amp = tf(Ka,[Ta 1]); G_exc = tf(Ke,[Te 1]); G_gen = tf(Kg,[Tg 1]); G_sen = tf(Ks,[Ts 1]);
G_fwd = minreal(G_amp * G_exc * G_gen);

Tfinal = 25; dt = 0.002; t = (0:dt:Tfinal)'; r = ones(size(t));
scenarios = {};
scenarios{end+1} = struct('name','bias_small','type','bias','magnitude',0.1,'start_time',5);
scenarios{end+1} = struct('name','bias_large','type','bias','magnitude',0.5,'start_time',5);
scenarios{end+1} = struct('name','ramp','type','ramp','slope',0.05,'start_time',5);
scenarios{end+1} = struct('name','sine','type','sine','magnitude',0.1,'frequency',1,'start_time',5);

for is = 1:numel(scenarios)
    sc = scenarios{is};
    attack_cfg = struct('enabled',true,'type',sc.type,'start_time',sc.start_time);
    if isfield(sc,'magnitude'), attack_cfg.magnitude = sc.magnitude; end
    if isfield(sc,'slope'), attack_cfg.slope = sc.slope; end
    if isfield(sc,'frequency'), attack_cfg.frequency = sc.frequency; end

    % Resilient run (with bumpless transfer code active)
    [u_res, mode_hist, switch_times, y_res, diag] = simulate_resilient_closedloop_euler( ss(G_fwd), ss(G_sen), C_2dof_r, C_2dof_y, pidtune(G_fwd*G_sen,'PID'), t, r, attack_cfg, 1, 5 );
    y_res = sanitize_signal(y_res);
    itae_res = safe_itae(y_res, t, 1e6);
    if isempty(switch_times)
        u_jump = NaN;
    else
        idx_sw = find(t >= switch_times(1,1), 1, 'first'); if isempty(idx_sw), idx_sw = numel(t); end
        u_prev_sw = u_res(max(1, idx_sw-1)); u_post_sw = u_res(min(numel(u_res), idx_sw)); u_jump = u_post_sw - u_prev_sw;
    end
    fprintf('%s: itae_res=%.4f u_jump=%.4f\n', sc.name, itae_res, u_jump);
end

fprintf('Bumpless test complete. For full comparison re-run Phase5 pipeline.\n');
