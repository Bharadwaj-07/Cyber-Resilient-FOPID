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
            try
                set(legs(k), 'TextColor', [0 0 0]);
            catch
            end
            try
                set(legs(k), 'Interpreter', 'tex');
            catch
            end
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
    % increase minimum exported figure size for better readability
    minW = 1500; minH = 1200;
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
                % Tiled layouts manage axes geometry themselves; avoid direct Position edits.
                continue;
            end
            % tighten position based on TightInset for standalone axes only
            pos = get(ax, 'Position');
            ti = get(ax, 'TightInset');
            % compute a slightly looser tight position to preserve breathing room
            left = pos(1) + ti(1);
            bottom = pos(2) + ti(2);
            width = pos(3) - (ti(1) + ti(3));
            height = pos(4) - (ti(2) + ti(4));
            % expand a bit to avoid clipping close to axis lines
            margin = 0.02; % normalized units
            left = max(0, left - margin);
            bottom = max(0, bottom - margin);
            width = min(1 - left, width + 2 * margin);
            height = min(1 - bottom, height + 2 * margin);
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