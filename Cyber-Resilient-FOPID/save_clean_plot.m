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