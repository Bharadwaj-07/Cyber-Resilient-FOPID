%% avr_validate_plant.m
% Extracts step response metrics and saves them as baseline
% Run avr_plant_model.m first

avr_parameters;
avr_plant_model;
paths = phase_artifacts('phase1');

% --- Step response info ---
info = stepinfo(G_cl);

fprintf('\n=== Plant baseline metrics (no controller) ===\n');
fprintf('Rise time:         %.4f s\n', info.RiseTime);
fprintf('Settling time:     %.4f s\n', info.SettlingTime);
fprintf('Overshoot:         %.2f %%\n', info.Overshoot);
fprintf('Undershoot:        %.2f %%\n', info.Undershoot);
fprintf('Peak:              %.4f\n',   info.Peak);
fprintf('Peak time:         %.4f s\n', info.PeakTime);

% --- Compute ITAE for the uncontrolled step response ---
t = 0:0.001:Tfinal;
[y, t_out] = step(G_cl, t);
e = 1 - y;                          % error = setpoint - output
ITAE = trapz(t_out, t_out .* abs(e));
fprintf('ITAE (no ctrl):    %.4f\n', ITAE);

% --- Save baseline for later comparison ---
baseline.RiseTime    = info.RiseTime;
baseline.SettlingTime= info.SettlingTime;
baseline.Overshoot   = info.Overshoot;
baseline.ITAE        = ITAE;
save(fullfile(paths.mat, 'avr_baseline.mat'), 'baseline');
disp(['Baseline saved to ' fullfile(paths.mat, 'avr_baseline.mat')]);

summary = table(info.RiseTime, info.SettlingTime, info.Overshoot, info.Undershoot, info.Peak, info.PeakTime, ITAE, ...
    'VariableNames', {'rise_time','settling_time','overshoot','undershoot','peak','peak_time','itae'});
writetable(summary, fullfile(paths.csv, 'avr_baseline_summary.csv'));

% --- Annotated step plot ---
hf = figure('Name','Baseline Step Response','Visible','off','Color','w');
plot(t_out, y, 'LineWidth', 1.5); hold on;
yline(1.0, '--k', 'Setpoint', 'LabelHorizontalAlignment','left');
yline(1 + info.Overshoot/100, ':r', ...
    sprintf('Peak +%.1f%%', info.Overshoot), ...
    'LabelHorizontalAlignment','left');
xline(info.SettlingTime, ':b', ...
    sprintf('Ts = %.2fs', info.SettlingTime));
grid on; hold off;
title('AVR plant — uncontrolled step response');
ylabel('Vt (pu)'); xlabel('Time (s)');
saveas(hf, fullfile(paths.plots, 'avr_baseline_step.png'));
close(hf);