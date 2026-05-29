function phase5_augmented_compare()
% Compare baseline vs augmented (resilient) versions of PID, 1DoF and 2DoF.
paths5 = phase_artifacts('phase5'); outcsv = fullfile(paths5.csv,'phase5_augmented_comparison.csv');
plotdir = fullfile(paths5.plots, 'augmented');
if ~exist(paths5.csv,'dir'), mkdir(paths5.csv); end
if ~exist(plotdir,'dir'), mkdir(plotdir); end

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

    % Augmented PID: use the same controller in both reference and feedback
    % paths so the resilient wrapper preserves the nominal PID behavior.
    [u_res_pid,~,switch_times_pid,y_res_pid,diag_pid] = simulate_resilient_closedloop_euler(ss(G_fwd), ss(G_sen), C_pid, C_pid, C_pid, t, r, attack_cfg, 1, 5, struct('blend_time',0.5,'isolation_tau',0.25,'observer_recovery_time',1.0,'actuator_limits',[-5 5]));
    y_res_pid = sanitize_signal(y_res_pid); itae_res_pid = safe_itae(y_res_pid,t,1e6);

    % Baseline 1DoF (feedback-only)
    y_1dof = simulate_closedloop_pid_euler_attacked(ss(G_fwd), ss(G_sen), C_1dof, t, r, attack_cfg);
    y_1dof = sanitize_signal(y_1dof); itae_1dof = safe_itae(y_1dof,t,1e6);

    % Augmented 1DoF: duplicate the controller into both 2DoF paths so the
    % resilient pipeline sees the same closed-loop shape as the baseline.
    [u_res_1d,~,switch_times_1d,y_res_1d,diag_1d] = simulate_resilient_closedloop_euler(ss(G_fwd), ss(G_sen), C_1dof, C_1dof, C_1dof, t, r, attack_cfg, 1, 5, struct('blend_time',0.5,'isolation_tau',0.25,'observer_recovery_time',1.0,'actuator_limits',[-5 5]));
    y_res_1d = sanitize_signal(y_res_1d); itae_res_1d = safe_itae(y_res_1d,t,1e6);

    % Baseline 2DoF
    [y_2dof,~] = simulate_closedloop_2dof_euler_attacked(ss(G_fwd), ss(G_sen), C_2dof_r, C_2dof_y, t, r, attack_cfg);
    y_2dof = sanitize_signal(y_2dof); itae_2dof = safe_itae(y_2dof,t,1e6);

    % Augmented 2DoF: use the same 2DoF structure with a stronger recovery profile.
    cfg_2d = struct('blend_time',0.25,'isolation_tau',0.15,'observer_recovery_time',0.6, ...
        'recovery_time',0.6,'actuator_limits',[-5 5],'use_attack_subtraction',true, ...
        'use_aggressive_obs_gain',true,'observer_min_gain',0.05);
    [u_res_2d,~,switch_times_2d,y_res_2d,diag_2d] = simulate_resilient_closedloop_euler(ss(G_fwd), ss(G_sen), C_2dof_r, C_2dof_y, C_pid, t, r, attack_cfg, 1, 5, cfg_2d);
    y_res_2d = sanitize_signal(y_res_2d); itae_res_2d = safe_itae(y_res_2d,t,1e6);

    % Save a per-scenario plot comparing baseline vs augmented responses.
    hf = figure('Visible','off','Color','w','Position',[120 80 1400 900]);
    tiledlayout(3,2,'Padding','compact','TileSpacing','compact');

    nexttile;
    plot(t, y_pid, 'b-', 'LineWidth', 1.1); hold on;
    plot(t, y_res_pid, 'r-', 'LineWidth', 1.1);
    grid on; title('PID: baseline vs augmented'); ylabel('y');
    legend('baseline PID','augmented PID','Location','best');
    shade_attack_window(gca, attack_cfg.start_time, t(end), [0.65 0.80 1.0], 0.16);
    xline(attack_cfg.start_time,'m-.','Attack start');

    nexttile;
    plot(t, abs(r - y_pid), 'b--', 'LineWidth', 1.0); hold on;
    plot(t, abs(r - y_res_pid), 'r-', 'LineWidth', 1.0);
    grid on; title('PID tracking error'); ylabel('|e|');
    legend('baseline PID','augmented PID','Location','best');
    shade_attack_window(gca, attack_cfg.start_time, t(end), [0.65 0.80 1.0], 0.16);
    xline(attack_cfg.start_time,'m-.','Attack start');

    nexttile;
    plot(t, y_1dof, 'b-', 'LineWidth', 1.1); hold on;
    plot(t, y_res_1d, 'r-', 'LineWidth', 1.1);
    grid on; title('1DoF: baseline vs augmented'); ylabel('y');
    legend('baseline 1DoF','augmented 1DoF','Location','best');
    shade_attack_window(gca, attack_cfg.start_time, t(end), [0.65 0.80 1.0], 0.16);
    xline(attack_cfg.start_time,'m-.','Attack start');

    nexttile;
    plot(t, abs(r - y_1dof), 'b--', 'LineWidth', 1.0); hold on;
    plot(t, abs(r - y_res_1d), 'r-', 'LineWidth', 1.0);
    grid on; title('1DoF tracking error'); ylabel('|e|');
    legend('baseline 1DoF','augmented 1DoF','Location','best');
    shade_attack_window(gca, attack_cfg.start_time, t(end), [0.65 0.80 1.0], 0.16);
    xline(attack_cfg.start_time,'m-.','Attack start');

    nexttile;
    plot(t, y_2dof, 'b-', 'LineWidth', 1.1); hold on;
    plot(t, y_res_2d, 'r-', 'LineWidth', 1.1);
    grid on; title('2DoF: baseline vs augmented'); xlabel('Time (s)'); ylabel('y');
    legend('baseline 2DoF','augmented 2DoF','Location','best');
    shade_attack_window(gca, attack_cfg.start_time, t(end), [0.65 0.80 1.0], 0.16);
    xline(attack_cfg.start_time,'m-.','Attack start');

    nexttile;
    plot(t, abs(r - y_2dof), 'b--', 'LineWidth', 1.0); hold on;
    plot(t, abs(r - y_res_2dof), 'r-', 'LineWidth', 1.0);
    grid on; title('2DoF tracking error'); xlabel('Time (s)'); ylabel('|e|');
    legend('baseline 2DoF','augmented 2DoF','Location','best');
    shade_attack_window(gca, attack_cfg.start_time, t(end), [0.65 0.80 1.0], 0.16);
    xline(attack_cfg.start_time,'m-.','Attack start');

    sgtitle(sprintf('Augmented comparison - %s', sc.name), 'Interpreter', 'none');
    plotfile = fullfile(plotdir, sprintf('%s_augmented_compare.png', sc.name));
    save_plot(hf, plotfile);
    close(hf);

    % Small summary bar plot for the scenario.
    hf2 = figure('Visible','off','Color','w','Position',[150 100 1100 500]);
    vals = [itae_pid, itae_res_pid; itae_1dof, itae_res_1d; itae_2dof, itae_res_2d];
    bar(vals);
    grid on; ylabel('ITAE');
    set(gca,'XTickLabel',{'PID','1DoF','2DoF'});
    legend('baseline','augmented','Location','northwest');
    title(sprintf('ITAE summary - %s', sc.name), 'Interpreter', 'none');
    plotfile2 = fullfile(plotdir, sprintf('%s_augmented_itae.png', sc.name));
    save_plot(hf2, plotfile2);
    close(hf2);

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

function y = sanitize_signal(y)
    y = y(:);
    y(~isfinite(y)) = NaN;
    if all(isnan(y))
        y = zeros(size(y));
        return;
    end
    firstValid = find(~isnan(y),1,'first');
    if firstValid > 1
        y(1:firstValid-1) = y(firstValid);
    end
    for k = 2:numel(y)
        if isnan(y(k))
            y(k) = y(k-1);
        end
    end
end

function val = safe_itae(y,t,cap)
    if any(~isfinite(y)) || max(abs(y)) > cap
        val = NaN;
        return;
    end
    e = abs(1 - y(:));
    val = trapz(t(:), t(:) .* e);
    if ~isfinite(val)
        val = NaN;
    end
end

function shade_attack_window(ax, attack_start, attack_end, faceColor, faceAlpha)
    if ~isfinite(attack_start) || ~isfinite(attack_end) || attack_end <= attack_start
        return;
    end
    axes(ax); %#ok<LAXES>
    yl = ylim(ax);
    hold(ax, 'on');
    hBand = patch(ax, [attack_start attack_end attack_end attack_start], [yl(1) yl(1) yl(2) yl(2)], faceColor, ...
        'FaceAlpha', faceAlpha, 'EdgeColor', 'none', 'HandleVisibility', 'off');
    uistack(hBand, 'bottom');
end

function save_plot(hf, filePath)
    try
        exportgraphics(hf, filePath, 'Resolution', 150);
    catch
        saveas(hf, filePath);
    end
end
