function save_clean_plot(hf, filePath, resolution)
%SAVE_CLEAN_PLOT Style and export a figure with a white background.

if nargin < 1 || isempty(hf) || ~isgraphics(hf, 'figure')
    hf = gcf;
end
if nargin < 3 || isempty(resolution)
    resolution = 300; % higher default for clearer saved figures
end

set(hf, 'Color', 'w');
axesHandles = findall(hf, 'Type', 'axes');
for k = 1:numel(axesHandles)
    apply_clean_plot_style(axesHandles(k));
end

drawnow;
% Try to move legends outside the axes to avoid overlap
try
    legs = findall(hf, 'Type', 'Legend');
    for k = 1:numel(legs)
        try
            set(legs(k), 'Interpreter', 'none');
            set(legs(k), 'Box', 'on', 'Color', 'w', 'EdgeColor', 0.85*[1 1 1]);
            set(legs(k), 'Location', 'bestoutside');
        catch
            try
                set(legs(k), 'Location', 'best');
            catch
            end
        end
    end
catch
end

% Automatic layout fixes: ensure figure is large enough and adjust axes
try
    % Ensure minimum figure size (pixels) for readability
    oldUnits = get(hf, 'Units');
    set(hf, 'Units', 'pixels');
    pos = get(hf, 'Position');
    minW = 1000; minH = 650;
    if pos(3) < minW || pos(4) < minH
        set(hf, 'Position', [pos(1) pos(2) max(pos(3), minW) max(pos(4), minH)]);
    end
    set(hf, 'Units', oldUnits);
catch
end

% For each axes, compact tiledlayout and tighten axes position to avoid overlaps
try
    axesHandles = findall(hf, 'Type', 'axes');
    for k = 1:numel(axesHandles)
        ax = axesHandles(k);
        try
            tl = ancestor(ax, 'tiledlayout');
            if ~isempty(tl)
                try
                    set(tl, 'TileSpacing', 'compact', 'Padding', 'compact');
                catch
                end
            end
            % tighten position based on TightInset
            pos = get(ax, 'Position');
            ti = get(ax, 'TightInset');
            left = pos(1) + ti(1);
            bottom = pos(2) + ti(2);
            width = pos(3) - (ti(1) + ti(3));
            height = pos(4) - (ti(2) + ti(4));
            if width > 0 && height > 0
                set(ax, 'Position', [left bottom max(width, 0.05) max(height, 0.05)]);
            end
        catch
        end
    end
catch
end

% Attempt export with white background and requested resolution
try
    exportgraphics(hf, filePath, 'Resolution', resolution, 'BackgroundColor', 'white');
catch
    try
        exportgraphics(hf, filePath, 'Resolution', resolution);
    catch
        saveas(hf, filePath);
    end
end
end