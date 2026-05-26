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

% Collect per-phase logs (phase5)
try
    p5dir = fullfile('results','phase5');
    if exist(p5dir,'dir')
        files = dir(fullfile(p5dir,'phase5_run_*.log'));
        for k=1:length(files)
            src = fullfile(p5dir, files(k).name);
            dest = fullfile(outRoot, files(k).name);
            copyfile(src, dest);
            fprintf(runfid, 'Copied %s to %s\n', src, dest);
        end
    end
catch ME
    fprintf(runfid, 'Error collecting phase5 logs: %s\n', ME.message);
end

% Collect Phase3 logs
try
    p3dir = fullfile('results','phase3');
    if exist(p3dir,'dir')
        files3 = dir(fullfile(p3dir,'phase3_quick_run_*.log'));
        files3b = dir(fullfile(p3dir,'phase3_run_*.log'));
        files3 = [files3; files3b];
        for k=1:length(files3)
            src = fullfile(p3dir, files3(k).name);
            dest = fullfile(outRoot, files3(k).name);
            copyfile(src, dest);
            fprintf(runfid, 'Copied %s to %s\n', src, dest);
        end
    end
    % Also collect logs from legacy phase3_results directory if present
    legacy_p3 = fullfile(pwd,'phase3_results');
    if exist(legacy_p3,'dir')
        files_legacy = dir(fullfile(legacy_p3,'phase3_run_*.log'));
        for k=1:length(files_legacy)
            src = fullfile(legacy_p3, files_legacy(k).name);
            dest = fullfile(outRoot, files_legacy(k).name);
            copyfile(src, dest);
            fprintf(runfid, 'Copied %s to %s\n', src, dest);
        end
    end
catch ME
    fprintf(runfid, 'Error collecting phase3 logs: %s\n', ME.message);
end

% Collect Phase4 logs
try
    p4dir = fullfile('results','phase4');
    if exist(p4dir,'dir')
        files4 = dir(fullfile(p4dir,'phase4_switcher_*.log'));
        for k=1:length(files4)
            src = fullfile(p4dir, files4(k).name);
            dest = fullfile(outRoot, files4(k).name);
            copyfile(src, dest);
            fprintf(runfid, 'Copied %s to %s\n', src, dest);
        end
    end
catch ME
    fprintf(runfid, 'Error collecting phase4 logs: %s\n', ME.message);
end
% Collect Phase4 state history files if present
try
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
        set(gca,'XTickLabel', T.scenario);
        legend('2DoF','PID','Resilient'); title('Phase5 ITAE Comparison'); ylabel('ITAE');
        saveas(hf,outfig); close(hf);
        fprintf('Saved Phase5 ITAE plot to %s\n', outfig);
    catch
        warning('Could not create phase5 ITAE plot');
    end
end

fprintf('All done. Logs and artifacts in results/run_all and results/phase5 if available.\n');
