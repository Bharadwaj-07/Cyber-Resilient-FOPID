function avr_phase3_plot(result, outname)
% AVR_PHASE3_PLOT Plot phase-3 results for one scenario
% avr_phase3_plot(result) or avr_phase3_plot('results/phase3_bias.mat')
%
% If `result` is a filename, loads struct `r` saved by avr_phase3_test. If `result` is a struct,
% uses it directly. `outname` optional PNG filename; saved to results/ if given.

if ischar(result) || isstring(result)
    data = load(result);
    if isfield(data,'r'), r = data.r; else r = data.results; end
else
    r = result;
end

t = 0:0.001: (length(r.y_true)-1)/1000;
if isfield(r,'y_meas'), y_meas = r.y_meas; else y_meas = r.y_true; end
if isfield(r,'residuals'), residuals = r.residuals; else residuals = zeros(size(t)); end
if isfield(r,'detection_time'), dt = r.detection_time; else dt = NaN; end
if isfield(r,'mode_history'), mode_history = r.mode_history; else mode_history = ones(size(t)); end
if isfield(r,'u_switched'), u = r.u_switched; else u = zeros(size(t)); end

figure('Units','normalized','Position',[0.1 0.1 0.7 0.6]);
ax1 = subplot(3,1,1);
plot(t, r.y_true, 'k-', 'LineWidth', 1.5); hold on;
plot(t, y_meas, 'r--');
yline(1.0, 'k:');
legend('y_{true}','y_{meas}','Setpoint','Location','southeast');
title(sprintf('Output — attack: %s', r.attack_type));
xlabel('Time (s)'); ylabel('V_{t} (pu)'); grid on;
if ~isnan(dt), xline(dt, 'm--', 'Detection'); end

ax2 = subplot(3,1,2);
plot(t, residuals, 'b-'); hold on; yline(0,'k:');
title('Residual (y_{meas}-\hat{y})'); xlabel('Time (s)'); ylabel('Residual'); grid on;
if ~isnan(dt), xline(dt,'m--','Detection'); end

ax3 = subplot(3,1,3);
plot(t, u, 'g-'); hold on;
stairs(t, mode_history, 'k--');
legend('u (control)','mode (1 normal,2 pid,3 rec)');
title('Control and Mode History'); xlabel('Time (s)'); ylabel('u / mode'); grid on;
if ~isnan(dt), xline(dt,'m--','Detection'); end

linkaxes([ax1 ax2 ax3],'x');
if nargin >= 2 && ~isempty(outname)
    % ensure results dir
    if ~exist('results','dir'), mkdir('results'); end
    saveas(gcf, outname);
end
end
