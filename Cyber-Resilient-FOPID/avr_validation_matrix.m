function avr_validation_matrix(result_files)
% AVR_VALIDATION_MATRIX Load Phase 3 result files and print comparison table
% avr_validation_matrix(result_files)
% If result_files not provided, loads 'results/phase3_all.mat'

if nargin < 1 || isempty(result_files)
    if exist('results/phase3_all.mat','file')
        data = load('results/phase3_all.mat');
        results = normalize_results(data.results);
    else
        error('No results found. Run avr_phase3_test.m first.');
    end
else
    results = normalize_results(result_files);
end

n = numel(results);
fprintf('\nPhase 3 Validation Matrix\n');
fprintf('Attack Type | Detected | Det Time(s) | Delay(s) | FP? | Modes | ITAE_2DoF | ITAE_Switched | ITAE_PID\n');
fprintf('-------------------------------------------------------------------------------------------------------\n');
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
    modes = NaN;
    if isfield(r,'mode_transitions')
        modes = r.mode_transitions;
    elseif isfield(r,'switch_times') && ~isempty(r.switch_times)
        modes = size(r.switch_times,1);
    end
    itae_2 = r.metrics.ITAE_2dof;
    itae_sw = r.metrics.ITAE_switched;
    itae_pid = r.metrics.ITAE_pid;
    fprintf('%10s   |   %1d     |   %8.3f | %7.3f | %3d | %5.0f | %8.4f |   %8.4f   |  %8.4f\n', ...
        r.attack_type, det, dt, delay, fp, modes, itae_2, itae_sw, itae_pid);
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
