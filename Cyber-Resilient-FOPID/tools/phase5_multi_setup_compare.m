function phase5_multi_setup_compare()
% Run PID, 1DoF, 2DoF and Resilient setups on the same scenarios and save CSV.
paths5 = phase_artifacts('phase5');
outcsv = fullfile(paths5.csv, 'phase5_multi_setup_comparison.csv');

% Load system and Phase2 controllers
avr_parameters;
phase2mat = fullfile(phase_artifacts('phase2').mat, 'avr_phase2.mat');
data = struct();
if exist(phase2mat,'file')
    data = load(phase2mat);
end
% Controller fallbacks
if isfield(data,'C_y'), C_2dof_y = data.C_y; end
if isfield(data,'C_r'), C_2dof_r = data.C_r; end
if isfield(data,'C_y_1dof'), C_1dof = data.C_y_1dof; end
if isfield(data,'C_pid'), C_pid = data.C_pid; end
G_amp = tf(Ka,[Ta 1]); G_exc = tf(Ke,[Te 1]); G_gen = tf(Kg,[Tg 1]); G_sen = tf(Ks,[Ts 1]);
G_fwd = minreal(G_amp * G_exc * G_gen);
if ~exist('C_2dof_y','var') || isempty(C_2dof_y)
    try C_2dof_y = pidtune(G_fwd * G_sen, 'PID'); C_2dof_r = C_2dof_y; catch, C_2dof_y = pid(1,1,0.1); C_2dof_r = C_2dof_y; end
end
if ~exist('C_pid','var') || isempty(C_pid)
    try, C_pid = pidtune(G_fwd * G_sen, 'PID'); catch, C_pid = pid(1,1,0.1); end
end
if ~exist('C_1dof','var') || isempty(C_1dof)
    C_1dof = C_pid;
end

% Time base and scenarios (shorter Tfinal for quick comparisons)
Tfinal = 25; dt = 0.002; t = (0:dt:Tfinal)'; r = ones(size(t));
scenarios = {};
scenarios{end+1} = struct('name','bias_small','type','bias','magnitude',0.1,'start_time',5);
scenarios{end+1} = struct('name','bias_large','type','bias','magnitude',0.5,'start_time',5);
scenarios{end+1} = struct('name','ramp','type','ramp','slope',0.05,'start_time',5);
scenarios{end+1} = struct('name','sine','type','sine','magnitude',0.1,'frequency',1,'start_time',5);

results = [];
row = 0;
for is = 1:numel(scenarios)
    sc = scenarios{is};
    attack_cfg = struct('enabled',true,'type',sc.type,'start_time',sc.start_time);
    if isfield(sc,'magnitude'), attack_cfg.magnitude = sc.magnitude; end
    if isfield(sc,'slope'), attack_cfg.slope = sc.slope; end
    if isfield(sc,'frequency'), attack_cfg.frequency = sc.frequency; end

    % PID (single-loop)
    y_pid = simulate_closedloop_pid_euler_attacked(ss(G_fwd), ss(G_sen), C_pid, t, r, attack_cfg);
    y_pid = sanitize_signal(y_pid);
    itae_pid = safe_itae(y_pid, t, 1e6);

    % 1DoF (feedback-only) - use C_1dof
    y_1dof = simulate_closedloop_pid_euler_attacked(ss(G_fwd), ss(G_sen), C_1dof, t, r, attack_cfg);
    y_1dof = sanitize_signal(y_1dof);
    itae_1dof = safe_itae(y_1dof, t, 1e6);

    % 2DoF
    [y_2dof, y_meas_hist] = simulate_closedloop_2dof_euler_attacked(ss(G_fwd), ss(G_sen), C_2dof_r, C_2dof_y, t, r, attack_cfg);
    y_2dof = sanitize_signal(y_2dof);
    itae_2dof = safe_itae(y_2dof, t, 1e6);

    % Resilient: detect from 2DoF baseline then run resilient sim
    [attack_flag, ~, detection_time, ~] = direct_baseline_detector(y_2dof, y_2dof, t, struct('baseline_window',5,'window_size',50,'threshold_factor',3,'min_consecutive',3,'startup_suppress',4.8));
    [u_res, mode_hist, switch_times, y_res, diag] = simulate_resilient_closedloop_euler( ss(G_fwd), ss(G_sen), C_2dof_r, C_2dof_y, C_pid, t, r, attack_cfg, attack_flag, detection_time, struct('blend_time',0.5,'isolation_tau',0.25,'observer_recovery_time',1.0,'actuator_limits',[-5 5]));
    y_res = sanitize_signal(y_res);
    itae_res = safe_itae(y_res, t, 1e6);

    % Simple metrics
    row = row + 1;
    results(row).scenario = sc.name;
    results(row).itae_pid = itae_pid; results(row).itae_1dof = itae_1dof; results(row).itae_2dof = itae_2dof; results(row).itae_res = itae_res;
    results(row).y_pid_final = safe_scalar(y_pid(end), NaN); results(row).y_1dof_final = safe_scalar(y_1dof(end), NaN);
    results(row).y_2dof_final = safe_scalar(y_2dof(end), NaN); results(row).y_res_final = safe_scalar(y_res(end), NaN);
    results(row).peak_pid = max(y_pid); results(row).peak_1dof = max(y_1dof); results(row).peak_2dof = max(y_2dof); results(row).peak_res = max(y_res);
    results(row).detection_time = detection_time; results(row).attack_flag = attack_flag;
end

% Save
T = struct2table(results);
if ~exist(paths5.csv,'dir'), mkdir(paths5.csv); end
writetable(T, outcsv);
fprintf('Wrote multi-setup comparison to %s\n', outcsv);
% attempt to generate plots using available per-scenario MATs
try
    create_multi_plots(T, t);
    fprintf('Saved multi-setup plots to phase5 artifacts and results/phase5/plots/multi\n');
catch
    fprintf('Could not generate multi-setup plots (missing MAT files)\n');
end
end

% Create per-scenario plots under Phase5 artifacts and top-level results
function create_multi_plots(results_table, t)
    paths5 = phase_artifacts('phase5');
    plotdir = fullfile(paths5.plots,'multi'); if ~exist(plotdir,'dir'), mkdir(plotdir); end
    results_plot_dir = fullfile('results','phase5','plots','multi'); if ~exist(results_plot_dir,'dir'), mkdir(results_plot_dir,'recursive'); end
    for i = 1:height(results_table)
        sc = results_table.scenario{i};
        % Load per-scenario MAT files if available
        try
            matf = fullfile(paths5.mat, [sc '.mat']);
            if exist(matf,'file')
                S = load(matf);
                y_pid = S.y_pid_sc; y_1d = S.y_1dof_sc; y_2d = S.y_2dof_sc; y_res = S.y_res;
            else
                % fallbacks: use table values (final only)
                y_pid = zeros(size(t)); y_1d = zeros(size(t)); y_2d = zeros(size(t)); y_res = zeros(size(t));
            end
        catch
            y_pid = zeros(size(t)); y_1d = zeros(size(t)); y_2d = zeros(size(t)); y_res = zeros(size(t));
        end
        hf = figure('Visible','off','Color','w');
        plot(t, y_1d, 'c', t, y_2d, 'b', t, y_pid, 'g', t, y_res, 'r');
        legend('1DoF','2DoF','PID','Resilient'); title(['Multi-setup - ' sc]); grid on;
        plotfile = fullfile(plotdir, [sc '_multi_compare.png']); save_plot(hf, plotfile); close(hf);
        try copyfile(plotfile, fullfile(results_plot_dir, [sc '_multi_compare.png'])); catch, end
    end
end

function save_plot(hf, filePath)
    try
        exportgraphics(hf, filePath, 'Resolution', 150);
    catch
        saveas(hf, filePath);
    end
end
