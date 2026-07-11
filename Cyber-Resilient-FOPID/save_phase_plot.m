function outPath = save_phase_plot(hf, phaseName, filename, resolution)
%SAVE_PHASE_PLOT Export a styled plot into a phase plot artifact.

if nargin < 4 || isempty(resolution)
    resolution = 300;
end

outPath = phase_artifact_file(phaseName, 'plots', filename);
save_clean_plot(hf, outPath, resolution);
end