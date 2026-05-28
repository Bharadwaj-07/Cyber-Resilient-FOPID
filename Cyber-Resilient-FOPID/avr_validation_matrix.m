function avr_validation_matrix(result_files)
% AVR_VALIDATION_MATRIX Load Phase 3 result files and print comparison table
% avr_validation_matrix(result_files)
% If result_files not provided, loads 'results/phase3_all.mat'

if nargin < 1 || isempty(result_files)
    paths3 = phase_artifacts('phase3');
    default_mat = fullfile(paths3.mat,'phase3_all.mat');
    if exist(default_mat,'file')
        data = load(default_mat);
        results = normalize_results(data.results);
    else
        error('No results found. Run avr_phase3_test.m first.');
    end
else
    results = normalize_results(result_files);
end

n = numel(results);
fprintf('\nPhase 3 Validation Matrix\n');
fprintf('Attack Type | Detected | Det Time(s) | Delay(s) | FP? | Residual RMS | Residual Peak\n');
fprintf('--------------------------------------------------------------------------------------\n');
rows = cell(n,1);
for i = 1:n
    r = results{i};
    det = r.attack_flag;
    dt = r.detection_time; if isempty(dt), dt = NaN; end
    delay = NaN;
    if isfield(r,'detection_delay') && ~isempty(r.detection_delay)
        delay = r.detection_delay;
    elseif isfield(r,'scenario') && isfield(r.scenario,'start_time') && ~isnan(dt)
        delay = dt - r.scenario.start_time;
    end
    fp = NaN;
    if isfield(r,'attack_config') && isfield(r.attack_config,'start_time') && ~isnan(dt)
        fp = double(dt < r.attack_config.start_time);
    end
    residual_rms = NaN;
    residual_peak = NaN;
    if isfield(r,'residual_rms'), residual_rms = r.residual_rms; end
    if isfield(r,'residual_peak'), residual_peak = r.residual_peak; end
    fprintf('%10s   |   %1d     |   %8.3f | %7.3f | %3d | %12.6f | %13.6f\n', ...
        r.attack_type, det, dt, delay, fp, residual_rms, residual_peak);
    rows{i} = struct('attack_type',r.attack_type,'detected',det,'detection_time',dt,'detection_delay',delay,'false_positive',fp,'residual_rms',residual_rms,'residual_peak',residual_peak);
end

try
    paths = phase_artifacts('phase3');
    outdir = paths.csv;
    writetable(struct2table([rows{:}]), fullfile(outdir, 'phase3_validation_matrix.csv'));
catch
end

end

function results = normalize_results(inputResults)
    if isstruct(inputResults)
        if isfield(inputResults,'runs')
            results = inputResults.runs;
        else
            results = {inputResults};
        end
    else
        results = inputResults;
    end

    if isstruct(results)
        results = num2cell(results);
    end
end
