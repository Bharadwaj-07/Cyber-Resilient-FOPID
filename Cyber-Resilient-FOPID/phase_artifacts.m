function paths = phase_artifacts(phaseName)
%PHASE_ARTIFACTS Create and return standard artifact folders for a phase.
%   paths = phase_artifacts('phase3') returns a struct with fields:
%     root, plots, mat, csv, logs

if nargin < 1 || isempty(phaseName)
    error('phase_artifacts requires a phase name');
end

phaseName = char(string(phaseName));
paths.root = fullfile(pwd, [phaseName '_results']);
paths.plots = fullfile(paths.root, 'plots');
paths.mat = fullfile(paths.root, 'mat');
paths.csv = fullfile(paths.root, 'csv');
paths.logs = fullfile(paths.root, 'logs');

if ~exist(paths.root, 'dir'), mkdir(paths.root); end
if ~exist(paths.plots, 'dir'), mkdir(paths.plots); end
if ~exist(paths.mat, 'dir'), mkdir(paths.mat); end
if ~exist(paths.csv, 'dir'), mkdir(paths.csv); end
if ~exist(paths.logs, 'dir'), mkdir(paths.logs); end
end