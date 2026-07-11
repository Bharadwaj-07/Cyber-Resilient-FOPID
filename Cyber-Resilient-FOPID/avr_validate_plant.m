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
write_phase_table('phase1', 'avr_baseline_summary.csv', summary);

% --- Annotated step plot ---
hf = figure('Name','Baseline Step Response','Visible','off','Color','w');
plot(t_out, y, 'Color', [0.0000 0.4470 0.7410], 'LineWidth', 1.6); hold on;
hsp = yline(1.0, '--k');
try set(hsp, 'DisplayName', 'Setpoint'); catch; end
hpeak = yline(1 + info.Overshoot/100, ':r');
try set(hpeak, 'DisplayName', sprintf('Peak +%.1f%%', info.Overshoot)); catch; end
% mark settling time with hidden xline and boxed label
xline(info.SettlingTime, ':b', 'HandleVisibility', 'off');
add_event_label(gca, info.SettlingTime, sprintf('Ts = %.2fs', info.SettlingTime), 'right');
grid on; hold off;
title('AVR plant — uncontrolled step response');
ylabel('Vt (pu)'); xlabel('Time (s)');
save_phase_plot(hf, 'phase1', 'avr_baseline_step.png');
close(hf);