function save_clean_plot(hf, filePath, resolution)
%SAVE_CLEAN_PLOT Style and export a figure with a white background.

if nargin < 1 || isempty(hf) || ~isgraphics(hf, 'figure')
    hf = gcf;
end
if nargin < 3 || isempty(resolution)
    resolution = 200;
end

set(hf, 'Color', 'w');
axesHandles = findall(hf, 'Type', 'axes');
for k = 1:numel(axesHandles)
    apply_clean_plot_style(axesHandles(k));
end

drawnow;
try
    exportgraphics(hf, filePath, 'Resolution', resolution);
catch
    saveas(hf, filePath);
end
end