function phase5_grid_search_recovery()
% Grid-search for recovery parameters and save results.
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
end
G_amp = tf(Ka,[Ta 1]); G_exc = tf(Ke,[Te 1]); G_gen = tf(Kg,[Tg 1]); G_sen = tf(Ks,[Ts 1]);
G_fwd = minreal(G_amp * G_exc * G_gen);

% Fallback controllers
if ~exist('C_2dof_y','var') || isempty(C_2dof_y)
    try, C_2dof_y = pidtune(G_fwd * G_sen, 'PID'); C_2dof_r = C_2dof_y; catch, C_2dof_y = pid(1,1,0.1); C_2dof_r = C_2dof_y; end
end
try, C_pid = pidtune(G_fwd * G_sen, 'PID'); catch, C_pid = pid(1,1,0.1); end

% Time base and scenarios
Tfinal = 25; dt = 0.001; t = (0:dt:Tfinal)'; r = ones(size(t));
scenarios = {};
scenarios{end+1} = struct('name','bias_small','type','bias','magnitude',0.1,'start_time',5);
scenarios{end+1} = struct('name','bias_large','type','bias','magnitude',0.5,'start_time',5);
scenarios{end+1} = struct('name','ramp','type','ramp','slope',0.05,'start_time',5);
scenarios{end+1} = struct('name','sine','type','sine','magnitude',0.1,'frequency',1,'start_time',5);

% Grid
blend_list = [0.5, 1.0, 1.5, 2.0];
recovery_list = [0.5, 1.0, 2.0];
reg_list = [1e-4, 1e-3, 1e-2];
act_limits = {[-2 2], [-5 5]};

results = []; row = 0;
for ib = 1:numel(blend_list)
    for ir = 1:numel(recovery_list)
        for ig = 1:numel(reg_list)
            for ia = 1:numel(act_limits)
                cfg = struct('blend_time',blend_list(ib),'recovery_time',recovery_list(ir),'bumpless_reg',reg_list(ig),'actuator_limits',act_limits{ia});
                for is = 1:numel(scenarios)
                    sc = scenarios{is};
                    attack_cfg = struct('enabled',true,'type',sc.type,'start_time',sc.start_time);
                    if isfield(sc,'magnitude'), attack_cfg.magnitude = sc.magnitude; end
                    if isfield(sc,'slope'), attack_cfg.slope = sc.slope; end
                    if isfield(sc,'frequency'), attack_cfg.frequency = sc.frequency; end

                    y_2dof_sc = simulate_closedloop_2dof_euler_attacked(ss(G_fwd), ss(G_sen), C_2dof_r, C_2dof_y, t, r, attack_cfg);
                    [attack_flag, ~, detection_time, ~] = direct_baseline_detector(y_2dof_sc, y_2dof_sc, t, struct('baseline_window',5,'window_size',50,'threshold_factor',3,'min_consecutive',3,'startup_suppress',4.8));
                    C_pid_tuned = C_pid;
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
                    results(row).scenario = sc.name; results(row).blend_time = cfg.blend_time; results(row).recovery_time = cfg.recovery_time; results(row).bumpless_reg = cfg.bumpless_reg; results(row).actuator_limits = cfg.actuator_limits;
                    results(row).itae_res = itae_res; results(row).y_res_final = safe_scalar(y_res(end),1e6); results(row).u_jump = u_jump; results(row).u_peak_rate = u_peak_rate;
                end
            end
        end
    end
end

% Save
T = struct2table(results);
if ~exist(paths5.csv,'dir'), mkdir(paths5.csv); end
writetable(T, outcsv);
if ~exist(paths5.mat,'dir'), mkdir(paths5.mat); end
save(outmat, 'results', 'T');
fprintf('Grid search complete: %s\n', outcsv);
end
