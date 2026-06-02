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

if isfield(r,'t') && ~isempty(r.t)
    t = r.t(:);
else
    t = (0:0.001:(length(r.y_true)-1)/1000)';
end
if isfield(r,'y_meas'), y_meas = r.y_meas; else y_meas = r.y_true; end
if isfield(r,'residuals'), residuals = r.residuals; else residuals = zeros(size(t)); end
if isfield(r,'detection_time'), dt = r.detection_time; else dt = NaN; end
if isfield(r,'residual_peak'), residual_peak = r.residual_peak; else residual_peak = NaN; end
if isfield(r,'residual_rms'), residual_rms = r.residual_rms; else residual_rms = NaN; end
attack_signal = y_meas - r.y_true;
attack_signal(~isfinite(attack_signal)) = 0;

if isfield(r,'attack_config') && isstruct(r.attack_config) && isfield(r.attack_config,'start_time')
    attack_start = r.attack_config.start_time;
else
    attack_start = NaN;
end

hf = figure('Units','normalized','Position',[0.05 0.06 0.90 0.78],'Color','w','Visible','on');
tiledlayout(3,1,'Padding','compact','TileSpacing','compact');

nexttile;
plot(t, r.y_true, 'Color', [0.0000 0.4470 0.7410], 'LineWidth', 1.5); hold on;
plot(t, y_meas, 'Color', [0.8500 0.3250 0.0980], 'LineWidth', 1.1);
yline(1.0, 'k:');
shade_attack_window(gca, attack_start, t(end), [0.65 0.80 1.0], 0.18);
% draw vertical event lines without labels, labels placed with add_event_label
if ~isnan(attack_start), xline(attack_start, 'b-.', 'HandleVisibility','off'); end
if ~isnan(dt), xline(dt, 'm--', 'HandleVisibility','off'); end
% Place legend outside to avoid overlapping with event labels
legend('y_{true}','y_{meas}','Setpoint','Location','northoutside','Orientation','horizontal');
title(sprintf('Phase 3 output and injection: %s', r.attack_type));
xlabel('Time (s)'); ylabel('V_t (pu)'); grid on;
% add boxed event labels (left/right) to avoid overlapping
ax = gca;
if ~isnan(attack_start), add_event_label(ax, attack_start, 'Attack start', 'left'); end
if ~isnan(dt), add_event_label(ax, dt, 'Detection', 'right'); end

nexttile;
plot(t, residuals, 'Color', [0.4940 0.1840 0.5560], 'LineWidth', 1.1); hold on; yline(0,'k:');
shade_attack_window(gca, attack_start, t(end), [0.65 0.80 1.0], 0.18);
if ~isnan(attack_start), xline(attack_start, 'b-.', 'HandleVisibility','off'); end
if ~isnan(dt), xline(dt,'m--','HandleVisibility','off'); end
title(sprintf('Residual | rms=%.4g | peak=%.4g', residual_rms, residual_peak)); xlabel('Time (s)'); ylabel('Residual'); grid on;
ax = gca;
% alternate label sides for clarity
if ~isnan(attack_start), add_event_label(ax, attack_start, 'Attack start', 'right'); end
if ~isnan(dt), add_event_label(ax, dt, 'Detection', 'left'); end

nexttile;
Jk = abs(residuals) + movmean(abs(residuals), max(5, round(0.05 / max(t(2)-t(1), eps))));
plot(t, Jk, 'Color', [0.4660 0.6740 0.1880], 'LineWidth', 1.1); hold on;
shade_attack_window(gca, attack_start, t(end), [0.65 0.80 1.0], 0.18);
if ~isnan(attack_start), xline(attack_start, 'b-.', 'HandleVisibility','off'); end
if ~isnan(dt), xline(dt,'m--','HandleVisibility','off'); end
legend('Detection metric','Location','best');
title('Detection metric'); xlabel('Time (s)'); ylabel('J_k'); grid on;
ax = gca;
if ~isnan(attack_start), add_event_label(ax, attack_start, 'Attack start', 'left'); end
if ~isnan(dt), add_event_label(ax, dt, 'Detection', 'right'); end

sgtitle(sprintf('Attack: %s | det=%s', r.attack_type, num2str(dt)));
if nargin >= 2 && ~isempty(outname)
    % ensure results dir
    outdir = fileparts(outname);
    if ~isempty(outdir) && ~exist(outdir,'dir'), mkdir(outdir); end
end
end

function shade_attack_window(ax, attack_start, attack_end, faceColor, faceAlpha)
    if isnan(attack_start) || ~isfinite(attack_start) || ~isfinite(attack_end) || attack_end <= attack_start
        return;
    end
    axes(ax); %#ok<LAXES>
    yl = ylim(ax);
    hold(ax, 'on');
    hBand = patch(ax, [attack_start attack_end attack_end attack_start], [yl(1) yl(1) yl(2) yl(2)], faceColor, ...
        'FaceAlpha', faceAlpha, 'EdgeColor', 'none', 'HandleVisibility', 'off');
    uistack(hBand, 'bottom');
end

function add_event_label(ax, xval, lab, side)
    % add_event_label Place a boxed label for vertical event at xval
    % side: 'left' or 'right'
    if nargin < 4 || isempty(side), side = 'right'; end
    try
        axes(ax); %#ok<LAXES>
        xl = xlim(ax); yl = ylim(ax);
        dx = (xl(2)-xl(1)) * 0.008;
        % y position slightly below top
        y = yl(2) - 0.04 * (yl(2)-yl(1));
        if strcmpi(side,'left')
            x = xval - dx;
            hal = 'right';
        else
            x = xval + dx;
            hal = 'left';
        end
        % ensure x within axis limits
        x = min(max(x, xl(1) + 0.005*(xl(2)-xl(1)), xl(1)), xl(2));
        t = text(ax, x, y, lab, 'HorizontalAlignment', hal, 'VerticalAlignment', 'top', ...
            'FontSize', 10, 'FontWeight', 'normal', 'Color', [0 0 0], 'Interpreter', 'none');
        set(t, 'BackgroundColor', [1 1 1], 'EdgeColor', 0.85*[1 1 1]);
    catch
    end
end
