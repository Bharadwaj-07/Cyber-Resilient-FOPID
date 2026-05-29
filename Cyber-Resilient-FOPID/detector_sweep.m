function detector_sweep()
% Sweep detector parameters and report detection times/false alarms for Phase3 scenarios
paths3 = phase_artifacts('phase3'); paths5 = phase_artifacts('phase5');
results_dir = paths5.mat;
if ~exist(results_dir,'dir'), mkdir(results_dir); end

avr_parameters;
phase2mat = fullfile(phase_artifacts('phase2').mat, 'avr_phase2.mat');
if exist(phase2mat,'file')
    data = load(phase2mat);
    if isfield(data,'C_y'), C_2dof_y = data.C_y; end
    if isfield(data,'C_r'), C_2dof_r = data.C_r; end
end
G_amp = tf(Ka,[Ta 1]); G_exc = tf(Ke,[Te 1]); G_gen = tf(Kg,[Tg 1]); G_sen = tf(Ks,[Ts 1]);
G_fwd = minreal(G_amp * G_exc * G_gen);

Tfinal = 25; dt = 0.001; t = (0:dt:Tfinal)'; r = ones(size(t));
scenarios = {};
scenarios{end+1} = struct('name','bias_small','type','bias','magnitude',0.1,'start_time',5);
scenarios{end+1} = struct('name','bias_large','type','bias','magnitude',0.5,'start_time',5);
scenarios{end+1} = struct('name','ramp','type','ramp','slope',0.05,'start_time',5);
scenarios{end+1} = struct('name','sine','type','sine','magnitude',0.1,'frequency',1,'start_time',5);

threshold_list = [2, 2.5, 3, 3.5, 4];
window_list = [20, 50, 100];
min_consecutive_list = [1, 3, 5];

out = [];
row = 0;
for th = threshold_list
    for win = window_list
        for mc = min_consecutive_list
            cfg = struct('baseline_window',5,'window_size',win,'threshold_factor',th,'min_consecutive',mc,'startup_suppress',4.8);
            for is = 1:numel(scenarios)
                sc = scenarios{is}; attack_cfg = struct('enabled',true,'type',sc.type,'start_time',sc.start_time);
                if isfield(sc,'magnitude'), attack_cfg.magnitude = sc.magnitude; end
                if isfield(sc,'slope'), attack_cfg.slope = sc.slope; end
                if isfield(sc,'frequency'), attack_cfg.frequency = sc.frequency; end
                y_2dof_sc = simulate_closedloop_2dof_euler_attacked(ss(G_fwd), ss(G_sen), C_2dof_r, C_2dof_y, t, r, attack_cfg);
                [attack_flag, confidence, detection_time, residuals] = direct_baseline_detector(y_2dof_sc, y_2dof_sc, t, cfg);
                row = row + 1;
                out(row).scenario = sc.name; out(row).threshold = th; out(row).window = win; out(row).min_consecutive = mc; out(row).detected = double(attack_flag); out(row).detection_time = detection_time; out(row).confidence = confidence;
            end
        end
    end
end
T = struct2table(out);
writetable(T, fullfile(results_dir,'detector_sweep_results.csv'));
save(fullfile(results_dir,'detector_sweep_results.mat'),'out','T');
fprintf('Detector sweep saved to %s\n', results_dir);
end
