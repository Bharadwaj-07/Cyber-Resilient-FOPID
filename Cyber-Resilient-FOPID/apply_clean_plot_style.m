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
        % Okabe-Ito colorblind-friendly palette (normalized RGB)
        0.0000 0.0000 0.0000; ... % black for baseline/first
        230/255 159/255 0/255; ... % orange
        86/255 180/255 233/255; ... % sky blue
        0/255 158/255 115/255; ... % bluish green
        240/255 228/255 66/255; ... % yellow
        0/255 114/255 178/255; ... % blue
        213/255 94/255 0/255; ... % vermillion
        204/255 121/255 167/255]);   % reddish purple
catch
end

if ~isempty(fig) && isgraphics(fig, 'figure')
    leg = findall(fig, 'Type', 'Legend');
    for k = 1:numel(leg)
        try
            % Prefer boxed legend with white background and subtle edge
            set(leg(k), 'FontName', 'Arial', 'FontSize', 11, ...
                'Box', 'on', 'Color', 'w', 'EdgeColor', 0.85 * [1 1 1], 'Interpreter', 'none');
            try
                set(leg(k), 'TextColor', [0 0 0]);
            catch
            end
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
        set(lines, 'LineWidth', 1.8);
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
% Ensure text is black for maximum contrast
try
    set([ax.Title, ax.XLabel, ax.YLabel], 'Color', [0 0 0]);
    set(ax, 'XColor', [0 0 0], 'YColor', [0 0 0]);
catch
end

% Reduce in-plot text size and avoid overlapping tick labels
try
    % Adjust generic text objects inside axes (not title/xlabel/ylabel)
    tx = findall(ax, 'Type', 'text');
    for t = tx(:)'
        try
            if ~isequal(t, get(ax, 'Title')) && ~isequal(t, get(ax, 'XLabel')) && ~isequal(t, get(ax, 'YLabel'))
                set(t, 'FontSize', 10, 'Interpreter', 'none');
            end
        catch
        end
    end
    % Rotate dense x-tick labels to avoid overlap
    xt = get(ax, 'XTick');
    if numel(xt) > 12
        try set(ax, 'XTickLabelRotation', 45); catch, end
    elseif numel(xt) > 8
        try set(ax, 'XTickLabelRotation', 30); catch, end
    end
    % Slightly increase loose inset to give labels room
    try
        ti = get(ax, 'TightInset');
        li = ti + 0.02; % small padding
        set(ax, 'LooseInset', li);
    catch
    end
catch
end
end