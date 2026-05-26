% run_all_phases.m
% Master runner: executes Phase 3 quick/full, Phase 4 checks, Phase 5 comparison,
% collects logs and generates summary plots. Writes results to results/run_all/

addpath(pwd);
outRoot = fullfile('results','run_all'); if ~exist(outRoot,'dir'), mkdir(outRoot); end
logfile = fullfile(outRoot,'run_log.txt'); runfid = fopen(logfile,'w');
closeLog = runfid > 2;
if runfid < 0
    warning('Could not open %s for writing; logging to console only.', logfile);
    runfid = 1;
end
global AVR_SHARED_LOG_FID AVR_SHARED_LOG_PATH
AVR_SHARED_LOG_FID = runfid;
AVR_SHARED_LOG_PATH = logfile;
fprintf(runfid,'Run all phases log - %s\n', datestr(now));

% Phase 3: quick test (if exists)
try
    if exist('phase3_quick_test.m','file')
        fprintf('Running phase3_quick_test...\n'); fprintf(runfid,'Running phase3_quick_test...\n');
        phase3_quick_test();
        fprintf(runfid,'phase3_quick_test completed\n');
    else
        fprintf(runfid,'phase3_quick_test not found - skipping\n'); fprintf(runfid,'phase3_quick_test not found - skipping\n');
    end
catch ME
    fprintf('Phase3 quick test failed: %s\n', ME.message);
    fprintf(runfid,'Phase3 quick test failed: %s\n', ME.message);
end

% Phase 3: full run
try
    if exist('phase3_full_run.m','file')
        fprintf('Running phase3_full_run...\n'); fprintf(runfid,'Running phase3_full_run...\n');
        phase3_full_run();
        fprintf(runfid,'phase3_full_run completed\n'); fprintf(runfid,'phase3_full_run completed\n');
    else
        fprintf('phase3_full_run not found - skipping\n'); fprintf(runfid,'phase3_full_run not found - skipping\n');
    end
catch ME
    fprintf('Phase3 full run failed: %s\n', ME.message); fprintf(runfid,'Phase3 full run failed: %s\n', ME.message);
end

% Phase 5: full comparison
try
    if exist('phase5_full_comparison.m','file')
        fprintf('Running phase5_full_comparison...\n'); fprintf(runfid,'Running phase5_full_comparison...\n');
        phase5_full_comparison();
        fprintf(runfid,'phase5_full_comparison completed\n'); fprintf(runfid,'phase5_full_comparison completed\n');
    else
        fprintf('phase5_full_comparison not found - skipping\n'); fprintf(runfid,'phase5_full_comparison not found - skipping\n');
    end
catch ME
    fprintf('Phase5 full comparison failed: %s\n', ME.message); fprintf(runfid,'Phase5 full comparison failed: %s\n', ME.message);
end

% Collect Phase4 state history files if present
try
    p4dir = fullfile('results','phase4');
    if exist(p4dir,'dir')
        files_s = dir(fullfile(p4dir,'phase4_states_*.mat'));
        for k=1:length(files_s)
            src = fullfile(p4dir, files_s(k).name);
            dest = fullfile(outRoot, files_s(k).name);
            copyfile(src, dest);
            fprintf(runfid, 'Copied %s to %s\n', src, dest);
        end
    end
catch ME
    fprintf(runfid, 'Error collecting phase4 state files: %s\n', ME.message);
end

if closeLog
    fclose(runfid);
end
fprintf('Run complete. See %s for details.\n', logfile);

% Simple aggregator: look for phase5 CSV and plot ITAE comparisons
csvpath = fullfile('results','phase5','phase5_comparison.csv');
if exist(csvpath,'file')
    T = readtable(csvpath);
    outfig = fullfile(outRoot,'phase5_ITAE.png');
    try
        hf = figure('Visible','off');
        bar([T.ITAE_2DoF, T.ITAE_PID, T.ITAE_Res]);
        set(gca,'XTickLabel', cellstr(string(T.scenario)));
        legend('2DoF','PID','Resilient'); title('Phase5 ITAE Comparison'); ylabel('ITAE');
        saveas(hf,outfig); close(hf);
        fprintf('Saved Phase5 ITAE plot to %s\n', outfig);
    catch
        warning('Could not create phase5 ITAE plot');
    end
end

fprintf('All done. Logs and artifacts in results/run_all and results/phase5 if available.\n');
