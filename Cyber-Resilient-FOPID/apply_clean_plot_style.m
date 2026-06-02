function apply_clean_plot_style(ax)
%APPLY_CLEAN_PLOT_STYLE Standardize figure styling for saved plots.

if nargin < 1 || isempty(ax) || ~isgraphics(ax, 'axes')
    ax = gca;
end

fig = ancestor(ax, 'figure');
if ~isempty(fig) && isgraphics(fig, 'figure')
    set(fig, 'Color', 'w');
end

set(ax, ...
    'Color', 'w', ...
    'Box', 'on', ...
    'Layer', 'top', ...
    'LineWidth', 1.5, ...
    'FontName', 'Arial', ...
    'FontSize', 12, ...
    'GridAlpha', 0.22, ...
    'MinorGridAlpha', 0.12, ...
    'XGrid', 'on', ...
    'YGrid', 'on', ...
    'XMinorGrid', 'on', ...
    'YMinorGrid', 'on', ...
    'TickDir', 'out', ...
    'TickLength', [0.02 0.02]);

try
    colororder(ax, [
        0.0000 0.4470 0.7410; ...
        0.8500 0.3250 0.0980; ...
        0.9290 0.6940 0.1250; ...
        0.4940 0.1840 0.5560; ...
        0.4660 0.6740 0.1880; ...
        0.3010 0.7450 0.9330]);
catch
end

if ~isempty(fig) && isgraphics(fig, 'figure')
    leg = findall(fig, 'Type', 'Legend');
    for k = 1:numel(leg)
        try
            % Prefer boxed legend with white background and subtle edge
            set(leg(k), 'FontName', 'Arial', 'FontSize', 11, ...
                'Box', 'on', 'Color', 'w', 'EdgeColor', 0.85 * [1 1 1], 'Interpreter', 'none');
            % Place legend outside if possible to avoid covering data
            try
                set(leg(k), 'Location', 'bestoutside', 'Orientation', 'vertical');
            catch
                set(leg(k), 'Location', 'best');
            end
        catch
        end
    end
end

% Improve line visuals (wider strokes, sensible marker sizes)
try
    lines = findall(ax, 'Type', 'line');
    if ~isempty(lines)
        set(lines, 'LineWidth', 1.6);
        try
            set(lines, 'MarkerSize', 6);
        catch
        end
    end
catch
end

% Ensure axis labels and title use plain interpreter to avoid LaTeX surprises
try
    th = get(ax, 'Title');
    set(th, 'Interpreter', 'none');
    xl = get(ax, 'XLabel');
    yl = get(ax, 'YLabel');
    set(xl, 'Interpreter', 'none');
    set(yl, 'Interpreter', 'none');
catch
end
end