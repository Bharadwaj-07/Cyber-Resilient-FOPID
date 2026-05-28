%% avr_compare_controllers.m
% Compares classical PID, 1DoF FOPID, and 2DoF FOPID on the AVR plant
% Requires avr_phase2.mat and avr_baseline.mat

avr_parameters;
paths = phase_artifacts('phase2');
load(fullfile(paths.mat, 'avr_phase2.mat'));
load(fullfile(phase_artifacts('phase1').mat, 'avr_baseline.mat'));

G_amp = tf(Ka,[Ta 1]); G_exc = tf(Ke,[Te 1]);
G_gen = tf(Kg,[Tg 1]); G_sen = tf(Ks,[Ts 1]);
G_fwd = G_amp * G_exc * G_gen;

t = 0 : 0.001 : Tfinal;

% --- 1. Classical PID (Ziegler-Nichols baseline when possible) ---
C_pid   = zn_pid(G_fwd * G_sen);
G_cl_pid = feedback(G_fwd * C_pid, G_sen);

% --- 2. 1DoF FOPID (dedicated tuning if available) ---
if exist('best_params_1dof', 'var')
    Kp=best_params_1dof(1); Ki=best_params_1dof(2); Kd=best_params_1dof(3);
    lam=best_params_1dof(4); mu=best_params_1dof(5);
    if exist('opts', 'var') && isfield(opts, 'frac')
        frac = opts.frac;
    else
        frac = struct('wb', 1e-2, 'wh', 1e2, 'N', 3);
    end
else
    % Fallback: reuse 2DoF parameters but force b=c=1
    Kp=best_params(1); Ki=best_params(2); Kd=best_params(3);
    lam=best_params(4); mu=best_params(5);
    frac = struct('wb', 1e-2, 'wh', 1e2, 'N', 3);
end
[C_r_1dof, C_y_1dof] = fopid_2dof(Kp, Ki, Kd, lam, mu, 1.0, 1.0, ...
    frac.wb, frac.wh, frac.N);
G_cl_1dof = minreal((G_fwd * C_r_1dof) / (1 + G_fwd * C_y_1dof * G_sen), 1e-3);

% --- 3. 2DoF FOPID (already computed) ---
% G_cl_2dof loaded from avr_phase2.mat

% --- Step responses ---
[y_pid,  t1] = step(G_cl_pid,  t);
[y_1dof, t2] = step(G_cl_1dof, t);
[y_2dof, t3] = step(G_cl_2dof, t);

% --- ITAE for each ---
itae = @(y,tv) trapz(tv, tv .* abs(1 - y));
ITAE_pid  = itae(y_pid,  t1);
ITAE_1dof = itae(y_1dof, t2);
ITAE_2dof = itae(y_2dof, t3);

% --- Comparison plot ---
hf = figure('Name','Controller Comparison','Position',[100 100 800 450],'Visible','off','Color','w');
plot(t1, y_pid,  'r-',  'LineWidth', 1.5); hold on;
plot(t2, y_1dof, 'b--', 'LineWidth', 1.5);
plot(t3, y_2dof, 'g-',  'LineWidth', 2.0);
yline(1.0, 'k:', 'Setpoint');
grid on; hold off;
legend('Classical PID', '1DoF FOPID', '2DoF FOPID (tuned)', ...
    'Location', 'southeast');
xlabel('Time (s)'); ylabel('Terminal voltage Vt (pu)');
title('AVR step response — controller comparison');
ylim([0 1.8]);
saveas(hf, fullfile(paths.plots, 'phase2_controller_comparison.png'));
close(hf);

% --- Metrics table ---
fprintf('\n%-20s %10s %12s %10s %10s\n', ...
    'Controller','Rise(s)','Settling(s)','OS(%%)','ITAE');
fprintf('%s\n', repmat('-',1,65));

controllers = {'Classical PID', '1DoF FOPID', '2DoF FOPID'};
cls = {G_cl_pid, G_cl_1dof, G_cl_2dof};
ITAEs = [ITAE_pid, ITAE_1dof, ITAE_2dof];

for i = 1:3
    inf_i = stepinfo(cls{i});
    fprintf('%-20s %10.4f %12.4f %10.2f %10.5f\n', ...
        controllers{i}, inf_i.RiseTime, inf_i.SettlingTime, ...
        inf_i.Overshoot, ITAEs(i));
end

% --- Save comparison ---
phase2_compare = table(controllers', ITAEs', 'VariableNames', {'controller_name','itae'});
writetable(phase2_compare, fullfile(paths.csv, 'phase2_controller_comparison.csv'));
save(fullfile(paths.mat, 'avr_comparison.mat'), 'ITAE_pid','ITAE_1dof','ITAE_2dof');
disp(['Comparison saved to ' fullfile(paths.mat, 'avr_comparison.mat')]);

% --- Helper: Ziegler-Nichols PID from gain margin ---
function C = zn_pid(G)
    [Gm, ~, ~, Wcg] = margin(G);
    if isempty(Gm) || isempty(Wcg) || ~isfinite(Gm) || ~isfinite(Wcg) || Gm <= 0 || Wcg <= 0
        C = pidtune(G, 'PID');
        return;
    end

    Ku = Gm;
    Pu = 2*pi/Wcg;
    Kp = 0.6*Ku;
    Ki = 1.2*Ku/Pu;
    Kd = 0.075*Ku*Pu;
    C = pid(Kp, Ki, Kd);
end