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

hf = figure('Units','normalized','Position',[0.08 0.08 0.84 0.74],'Color','w','Visible','off');
tiledlayout(3,1,'Padding','compact','TileSpacing','compact');

nexttile;
plot(t, r.y_true, 'k-', 'LineWidth', 1.4); hold on;
plot(t, y_meas, 'r-', 'LineWidth', 1.0);
yline(1.0, 'k:');
if ~isnan(dt), xline(dt, 'm--', 'Detection'); end
legend('y_{true}','y_{meas}','Setpoint','Detection','Location','best');
title(sprintf('Phase 3 output: %s', r.attack_type));
xlabel('Time (s)'); ylabel('V_t (pu)'); grid on;

nexttile;
plot(t, residuals, 'b-', 'LineWidth', 1.0); hold on; yline(0,'k:');
if ~isnan(dt), xline(dt,'m--','Detection'); end
title('Residual'); xlabel('Time (s)'); ylabel('Residual'); grid on;

nexttile;
plot(t, u, 'g-', 'LineWidth', 1.0); hold on;
stairs(t, mode_history, 'k--', 'LineWidth', 1.0);
if ~isnan(dt), xline(dt,'m--','Detection'); end
legend('u (control)','mode','Location','best');
title('Control and mode history'); xlabel('Time (s)'); ylabel('u / mode'); grid on;

sgtitle(sprintf('Attack: %s | det=%s', r.attack_type, num2str(dt)));
if nargin >= 2 && ~isempty(outname)
    % ensure results dir
    outdir = fileparts(outname);
    if ~isempty(outdir) && ~exist(outdir,'dir'), mkdir(outdir); end
    saveas(hf, outname);
end
close(hf);
end
