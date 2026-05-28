%% avr_closedloop_2dof.m
% Runs PSO tuning and builds the optimised 2DoF FOPID closed-loop
% Requires: avr_parameters.m, fopid_operator.m, fopid_2dof.m, pso_tuner.m

avr_parameters;
paths = phase_artifacts('phase2');

% --- Reproducibility ---
rng(1, 'twister');

% --- Rebuild plant blocks ---
G_amp = tf(Ka, [Ta 1]);
G_exc = tf(Ke, [Te 1]);
G_gen = tf(Kg, [Tg 1]);
G_sen = tf(Ks, [Ts 1]);
G_fwd = G_amp * G_exc * G_gen;

% --- PSO options (2DoF is the primary tuned controller per roadmap) ---
opts.n_particles = 40;
opts.max_iter    = 150;
opts.local_refine = false;
opts.local_maxiter = 0;
opts.Tfinal      = max(Tfinal, 25);
% Fractional operator settings (reduced order for stability)
opts.frac.wb = 1e-3;
opts.frac.wh = 1e3;
opts.frac.N  = 5;

% Allow caller to pass a struct 'robust_opts' to enable robust tuning
if exist('robust_opts','var') && isstruct(robust_opts)
    if isfield(robust_opts,'robust_mode'), opts.robust_mode = robust_opts.robust_mode; end
    if isfield(robust_opts,'robust_weight'), opts.robust_weight = robust_opts.robust_weight; end
    if isfield(robust_opts,'n_particles'), opts.n_particles = robust_opts.n_particles; end
    if isfield(robust_opts,'max_iter'), opts.max_iter = robust_opts.max_iter; end
end

% PID baseline to center bounds
C_pid0 = pidtune(G_fwd * G_sen, 'PID');
Kp0 = max(C_pid0.Kp, 0.2);
Ki0 = max(C_pid0.Ki, 0.2);
Kd0 = max(C_pid0.Kd, 0.05);

% Bounds centered around PID baseline but widened enough to let the optimizer
% move away from the classical PID solution when the fractional structure helps.
opts.bounds.lb   = [ ...
    0.05*Kp0 ...
    0.05*Ki0 ...
    0.005*Kd0 ...
    0.2 ...
    0.2 ...
    0.05 ...
    0.05 ];

opts.bounds.ub   = [ ...
    3.0*Kp0 ...
    3.0*Ki0 ...
    1.5*Kd0 ...
    1.8 ...
    1.8 ...
    1.0 ...
    1.0 ];

% Seed PSO away from the PID-like corner so the 2DoF run can actually explore
% a distinct reference weighting shape.
opts.seed = [0.8*Kp0, 0.8*Ki0, 0.5*Kd0, 0.95, 0.95, 0.35, 0.35];
% Evaluation targets: use a true ITAE objective for Phase 2 tuning; keep only
% hard stability rejection in the tuner so the reported value is meaningful.
opts.eval.settle_threshold = 0.02;
opts.eval.max_settle       = 5;
opts.eval.max_rise         = 1.5;
opts.eval.settle_weight    = 50;
opts.eval.rise_weight      = 20;
opts.eval.ss_weight        = 200;
opts.eval.target_os        = [];
opts.eval.os_weight        = 0;
opts.eval.objective_mode   = 'itae_only';

opts.w  = 0.72;
opts.c1 = 1.49;
opts.c2 = 1.49;

% --- Run tuner ---
tic;
[best_params, best_ITAE, pso_history] = pso_tuner(G_fwd, G_sen, opts);
elapsed = toc;
fprintf('2DoF tuning time: %.1f s\n', elapsed);

% --- Unpack results ---
Kp  = best_params(1);  Ki  = best_params(2);  Kd  = best_params(3);
lam = best_params(4);  mu  = best_params(5);
b   = best_params(6);  c   = best_params(7);

% --- Build optimised controller ---
[C_r, C_y] = fopid_2dof(Kp, Ki, Kd, lam, mu, b, c, ...
    opts.frac.wb, opts.frac.wh, opts.frac.N);

% --- Closed-loop transfer function ---
G_cl_2dof = minreal((G_fwd * C_r) / (1 + G_fwd * C_y * G_sen), 1e-3);

% --- Step response metrics ---
t = 0 : 0.001 : opts.Tfinal;
[y_2dof, t_2dof] = step(G_cl_2dof, t);
info_2dof = stepinfo(y_2dof, t_2dof, 'SettlingTimeThreshold', opts.eval.settle_threshold);

fprintf('\n=== 2DoF FOPID step response metrics ===\n');
fprintf('Rise time:     %.4f s\n', info_2dof.RiseTime);
fprintf('Settling time: %.4f s\n', info_2dof.SettlingTime);
fprintf('Overshoot:     %.2f %%\n', info_2dof.Overshoot);
fprintf('ITAE:          %.5f\n', best_ITAE);

% --- 1DoF comparison only (do not let it drive the main controller path) ---
opts_1dof = opts;
opts_1dof.fixed_bc = true;
opts_1dof.eval.objective_mode = 'itae_only';
opts_1dof.bounds.lb = [0.05*Kp0, 0.05*Ki0, 0.005*Kd0, 0.2, 0.2];
opts_1dof.bounds.ub = [3.0*Kp0, 3.0*Ki0, 1.5*Kd0, 1.8, 1.8];
opts_1dof.seed = [0.8*Kp0, 0.8*Ki0, 0.5*Kd0, 0.95, 0.95];

tic;
[best_params_1dof, best_ITAE_1dof, pso_history_1dof] = pso_tuner(G_fwd, G_sen, opts_1dof);
elapsed_1dof = toc;
fprintf('1DoF tuning time: %.1f s\n', elapsed_1dof);

Kp1  = best_params_1dof(1);  Ki1  = best_params_1dof(2);  Kd1  = best_params_1dof(3);
lam1 = best_params_1dof(4);  mu1  = best_params_1dof(5);

[C_r_1dof, C_y_1dof] = fopid_2dof(Kp1, Ki1, Kd1, lam1, mu1, 1.0, 1.0, ...
    opts.frac.wb, opts.frac.wh, opts.frac.N);
G_cl_1dof = minreal((G_fwd * C_r_1dof) / (1 + G_fwd * C_y_1dof * G_sen), 1e-3);
[y_1dof, t_1dof] = step(G_cl_1dof, t);
info_1dof = stepinfo(y_1dof, t_1dof, 'SettlingTimeThreshold', opts.eval.settle_threshold);

fprintf('\n=== 1DoF FOPID step response metrics ===\n');
fprintf('Rise time:     %.4f s\n', info_1dof.RiseTime);
fprintf('Settling time: %.4f s\n', info_1dof.SettlingTime);
fprintf('Overshoot:     %.2f %%\n', info_1dof.Overshoot);
fprintf('ITAE:          %.5f\n', best_ITAE_1dof);

% --- PSO convergence plot ---
hf = figure('Name','PSO Convergence','Visible','off','Color','w');
semilogy(pso_history, 'LineWidth', 1.5);
grid on;
xlabel('Iteration'); ylabel('Best ITAE (log scale)');
title('PSO convergence — 2DoF FOPID tuning');
saveas(hf, fullfile(paths.plots, 'phase2_pso_convergence.png'));
close(hf);

phase2_table = table(...
    {'2DoF'; '1DoF'}, ...
    [best_ITAE; best_ITAE_1dof], ...
    [info_2dof.RiseTime; info_1dof.RiseTime], ...
    [info_2dof.SettlingTime; info_1dof.SettlingTime], ...
    [info_2dof.Overshoot; info_1dof.Overshoot], ...
    'VariableNames', {'controller_name','itae','rise_time','settling_time','overshoot'});
writetable(phase2_table, fullfile(paths.csv, 'phase2_summary.csv'));

% --- Save ---
save(fullfile(paths.mat, 'avr_phase2.mat'), 'best_params', 'best_ITAE', ...
    'C_r', 'C_y', 'G_cl_2dof', 'pso_history', 'info_2dof', ...
    'best_params_1dof', 'best_ITAE_1dof', 'pso_history_1dof', ...
    'C_r_1dof', 'C_y_1dof', 'G_cl_1dof', 'info_1dof', ...
    'C_pid0', 'Kp0', 'Ki0', 'Kd0', 'opts');
disp(['Phase 2 results saved to ' fullfile(paths.mat, 'avr_phase2.mat')]);