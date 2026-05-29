function phase5_evaluate_top3_from_grid()
% Evaluate top-3 grid configs with per-scenario diagnostics and pick robust config
paths5 = phase_artifacts('phase5');
csvp = fullfile(paths5.csv, 'phase5_grid_search_results.csv');
matp = fullfile(paths5.mat, 'phase5_evaluator_results.mat');
outcsv = fullfile(paths5.csv, 'phase5_evaluator_results.csv');

if ~exist(csvp,'file')
    error('Grid results not found: %s', csvp);
end

T = readtable(csvp);

% Build a unique key per config
n = height(T);
keys = strings(n,1);
for i=1:n
    % assemble actuator limits for key (support two CSV formats)
    if ismember('actuator_limits', T.Properties.VariableNames)
        try
            lim = T.actuator_limits{i};
            a_limits = sprintf('%g_%g', lim(1), lim(2));
        catch
            a_limits = num2str(T.actuator_limits(i));
        end
    elseif ismember('actuator_limits_1', T.Properties.VariableNames) && ismember('actuator_limits_2', T.Properties.VariableNames)
        a_limits = sprintf('%g_%g', T.actuator_limits_1(i), T.actuator_limits_2(i));
    else
        a_limits = 'na';
    end
    keys(i) = sprintf('b%g_r%g_i%g_q%g_R%g_a%s', T.blend_time(i), T.recovery_time(i), T.isolation_tau(i), T.Q_scale(i), T.R_scale(i), a_limits);
end

uniq_keys = unique(keys);
agg = struct(); ai = 0;
for k = 1:numel(uniq_keys)
    idx = keys==uniq_keys(k);
    ai = ai + 1;
    agg(ai).key = uniq_keys(k);
    agg(ai).rows = find(idx)';
    agg(ai).mean_itae = mean(T.itae_res(idx),'omitnan');
    agg(ai).mean_u_jump = mean(T.u_jump(idx),'omitnan');
    agg(ai).mean_u_peak_rate = mean(T.u_peak_rate(idx),'omitnan');
    r = find(idx,1,'first');
    agg(ai).blend_time = T.blend_time(r);
    agg(ai).recovery_time = T.recovery_time(r);
    agg(ai).isolation_tau = T.isolation_tau(r);
    if ismember('actuator_limits', T.Properties.VariableNames)
            agg(ai).actuator_limits = T.actuator_limits{r};
        elseif ismember('actuator_limits_1', T.Properties.VariableNames) && ismember('actuator_limits_2', T.Properties.VariableNames)
            agg(ai).actuator_limits = [T.actuator_limits_1(r), T.actuator_limits_2(r)];
    else
        agg(ai).actuator_limits = [-inf inf];
    end
    agg(ai).Q_scale = T.Q_scale(r);
    agg(ai).R_scale = T.R_scale(r);
end

% Normalize aggregated metrics and compute score (lower is better)
M = [ [agg.mean_itae]' , [agg.mean_u_jump]' , [agg.mean_u_peak_rate]' ];
mn = nanmin(M,[],1); mx = nanmax(M,[],1); rngs = max(mx-mn, eps);
Mnorm = (M - mn) ./ rngs; score = sum(Mnorm,2);
[~, ord] = sort(score);
topk = min(3, numel(ord)); top_idx = ord(1:topk);

fprintf('Top-%d configs selected from grid for deeper evaluation.\n', topk);

% Load system/controllers like grid script
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

% Timebase and scenarios (same as grid)
Tfinal = 25; dt = 0.001; t = (0:dt:Tfinal)'; rref = ones(size(t));
scenarios = {};
scenarios{end+1} = struct('name','bias_small','type','bias','magnitude',0.1,'start_time',5);
scenarios{end+1} = struct('name','bias_large','type','bias','magnitude',0.5,'start_time',5);
scenarios{end+1} = struct('name','ramp','type','ramp','slope',0.05,'start_time',5);
scenarios{end+1} = struct('name','sine','type','sine','magnitude',0.1,'frequency',1,'start_time',5);

eval_results = [];
erow = 0;
for kk = 1:numel(top_idx)
    cfg_idx = top_idx(kk);
    cfg = struct('blend_time', agg(cfg_idx).blend_time, 'recovery_time', agg(cfg_idx).recovery_time, 'bumpless_reg', 1e-3, 'isolation_tau', agg(cfg_idx).isolation_tau, 'Q_scale', agg(cfg_idx).Q_scale, 'R_scale', agg(cfg_idx).R_scale, 'actuator_limits', agg(cfg_idx).actuator_limits);
    for is = 1:numel(scenarios)
        sc = scenarios{is};
        attack_cfg = struct('enabled',true,'type',sc.type,'start_time',sc.start_time);
        if isfield(sc,'magnitude'), attack_cfg.magnitude = sc.magnitude; end
        if isfield(sc,'slope'), attack_cfg.slope = sc.slope; end
        if isfield(sc,'frequency'), attack_cfg.frequency = sc.frequency; end

        y_2dof_sc = simulate_closedloop_2dof_euler_attacked(ss(G_fwd), ss(G_sen), C_2dof_r, C_2dof_y, t, rref, attack_cfg);
        [attack_flag, ~, detection_time, ~] = direct_baseline_detector(y_2dof_sc, y_2dof_sc, t, struct('baseline_window',5,'window_size',50,'threshold_factor',3,'min_consecutive',3,'startup_suppress',4.8));
        [u_res, mode_hist, switch_times, y_res, diag] = simulate_resilient_closedloop_euler( ss(G_fwd), ss(G_sen), C_2dof_r, C_2dof_y, C_pid, t, rref, attack_cfg, attack_flag, detection_time, cfg);
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

        erow = erow + 1;
        eval_results(erow).config_key = char(agg(cfg_idx).key);
        eval_results(erow).config_rank = kk;
        eval_results(erow).blend_time = cfg.blend_time;
        eval_results(erow).recovery_time = cfg.recovery_time;
        eval_results(erow).isolation_tau = cfg.isolation_tau;
        eval_results(erow).Q_scale = cfg.Q_scale;
        eval_results(erow).R_scale = cfg.R_scale;
        eval_results(erow).scenario = sc.name;
        eval_results(erow).itae = itae_res;
        eval_results(erow).u_jump = u_jump;
        eval_results(erow).u_peak_rate = u_peak_rate;
    end
end

% Save evaluator results
if ~exist(paths5.mat,'dir'), mkdir(paths5.mat); end
save(matp, 'eval_results', 'agg', 'top_idx');
T_eval = struct2table(eval_results);
if ~exist(paths5.csv,'dir'), mkdir(paths5.csv); end
writetable(T_eval, outcsv);
fprintf('Evaluator complete: %s\n', outcsv);

% Choose most robust among top-3 using worst-case normalized metric across scenarios
E = T_eval;
metrics = [E.itae, E.u_jump, E.u_peak_rate];
metrics = double(metrics);
mn = nanmin(metrics,[],1); mx = nanmax(metrics,[],1); rngs = max(mx-mn, eps);
metrics_norm = (metrics - mn) ./ rngs;

cfg_keys = unique(E.config_key);
cfg_score = zeros(numel(cfg_keys),1);
for i=1:numel(cfg_keys)
    idx = strcmp(E.config_key, cfg_keys{i});
    sub = metrics_norm(idx,:);
    cfg_score(i) = max(sub(:));
end
[~, besti] = min(cfg_score);
best_key = cfg_keys{besti};
fprintf('Best config chosen: %s\n', best_key);

% Save locked config info
best_cfg = [];
for i=1:numel(agg)
    if strcmp(char(agg(i).key), best_key)
        best_cfg.blend_time = agg(i).blend_time;
        best_cfg.recovery_time = agg(i).recovery_time;
        best_cfg.isolation_tau = agg(i).isolation_tau;
        best_cfg.actuator_limits = agg(i).actuator_limits;
        best_cfg.Q_scale = agg(i).Q_scale;
        best_cfg.R_scale = agg(i).R_scale;
        break;
    end
end

lockedp = fullfile(paths5.mat, 'phase5_locked.mat');
save(lockedp, 'best_cfg');
fprintf('Locked config saved: %s\n', lockedp);
end
