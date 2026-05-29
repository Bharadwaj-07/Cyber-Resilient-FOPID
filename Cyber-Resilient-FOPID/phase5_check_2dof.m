function phase5_check_2dof()
% Quick diagnostics for 2-DoF controllers used in Phase5
paths2 = phase_artifacts('phase2');
phase2mat = fullfile(paths2.mat, 'avr_phase2.mat');
fprintf('Checking Phase-2 controllers (file: %s)\n', phase2mat);
if exist(phase2mat,'file')
    data = load(phase2mat);
else
    warning('Phase2 mat not found: %s', phase2mat);
    data = struct();
end

% Try known field names
if isfield(data,'C_y'), C_2dof_y = data.C_y; end
if isfield(data,'C_y_2dof'), C_2dof_y = data.C_y_2dof; end
if ~exist('C_2dof_y','var'), C_2dof_y = []; end

if isfield(data,'C_r'), C_2dof_r = data.C_r; end
if isfield(data,'C_r_2dof'), C_2dof_r = data.C_r_2dof; end
if ~exist('C_2dof_r','var'), C_2dof_r = []; end

if isfield(data,'C_y_1dof'), C_pid = data.C_y_1dof; end
if ~exist('C_pid','var')
    C_pid = [];
end

report_ctrl('C_2dof_y', C_2dof_y);
report_ctrl('C_2dof_r', C_2dof_r);
report_ctrl('C_pid', C_pid);

% Short baseline sim if plant params available
try
    avr_parameters;
    G_amp = tf(Ka,[Ta 1]); G_exc = tf(Ke,[Te 1]); G_gen = tf(Kg,[Tg 1]); G_sen = tf(Ks,[Ts 1]);
    G_fwd = G_amp * G_exc * G_gen;
    Tfinal = 5; dt = 0.002; t = (0:dt:Tfinal)'; r = ones(size(t));
    fprintf('Running short baseline sim: dt=%.4f Tfinal=%.1f\n', dt, Tfinal);
    if ~isempty(C_2dof_r) && ~isempty(C_2dof_y)
        % Use the standalone attacked simulator with attacks disabled
        attack_cfg = struct('enabled', false);
        [y2, ~] = simulate_closedloop_2dof_euler_attacked(ss(G_fwd), ss(G_sen), C_2dof_r, C_2dof_y, t, r, attack_cfg);
        fprintf('2DoF final y = %.4f\n', y2(end));
    else
        fprintf('Skipping 2DoF sim (controllers missing)\n');
    end
    if ~isempty(C_pid)
        attack_cfg = struct('enabled', false);
        ypid = simulate_closedloop_pid_euler_attacked(ss(G_fwd), ss(G_sen), C_pid, t, r, attack_cfg);
        fprintf('PID final y = %.4f\n', ypid(end));
    else
        fprintf('Skipping PID sim (controller missing)\n');
    end
catch ME
    warning('Baseline sim failed: %s', ME.message);
end
end

function report_ctrl(name, C)
    fprintf('\nController: %s\n', name);
    if isempty(C)
        fprintf('  <missing>\n');
        return;
    end
    try
        if isa(C,'tf') || isa(C,'pid') || isa(C,'zpk')
            tfC = tf(C);
        else
            tfC = tf(C);
        end
        p = pole(tfC); z = zero(tfC);
        fprintf('  Type: %s\n', class(C));
        g0 = real(evalfr(tfC,0));
        fprintf('  DC gain: %.6g\n', g0);
        fprintf('  Poles: %s\n', mat2str(round(p.',6)));
        fprintf('  Zeros: %s\n', mat2str(round(z.',6)));
    catch ME
        fprintf('  Failed to inspect controller: %s\n', ME.message);
    end
end
