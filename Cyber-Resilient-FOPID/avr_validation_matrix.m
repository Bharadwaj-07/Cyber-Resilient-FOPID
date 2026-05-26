function avr_validation_matrix(result_files)
% AVR_VALIDATION_MATRIX Load Phase 3 result files and print comparison table
% avr_validation_matrix(result_files)
% If result_files not provided, loads 'results/phase3_all.mat'

if nargin < 1 || isempty(result_files)
    if exist('results/phase3_all.mat','file')
        data = load('results/phase3_all.mat');
        results = data.results;
    else
        error('No results found. Run avr_phase3_test.m first.');
    end
else
    results = result_files;
end

n = numel(results);
fprintf('\nPhase 3 Validation Matrix\n');
fprintf('Attack Type | Detected | Detection Time(s) | ITAE_2DoF | ITAE_Switched | ITAE_PID\n');
fprintf('-----------------------------------------------------------------------------------\n');
for i = 1:n
    r = results{i};
    det = r.attack_flag;
    dt = r.detection_time; if isempty(dt), dt = NaN; end
    itae_2 = r.metrics.ITAE_2dof;
    itae_sw = r.metrics.ITAE_switched;
    itae_pid = r.metrics.ITAE_pid;
    fprintf('%10s   |   %1d     |     %6.3f        |  %8.4f |   %8.4f   |  %8.4f\n', r.attack_type, det, dt, itae_2, itae_sw, itae_pid);
end

end
