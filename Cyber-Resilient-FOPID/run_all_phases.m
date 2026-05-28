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

% Phase 1: validate plant
try
    if exist('avr_validate_plant.m','file')
        fprintf('Running avr_validate_plant (Phase 1)...\n'); fprintf(runfid,'Running avr_validate_plant...\n');
        avr_validate_plant();
        fprintf(runfid,'avr_validate_plant completed\n');
    else
        fprintf(runfid,'avr_validate_plant not found - skipping Phase 1\n');
    end
catch ME
    fprintf(runfid,'Phase1 failed: %s\n', ME.message);
end

% Phase 2: tuning and controller comparison
try
    if exist('avr_closedloop_2dof.m','file')
        fprintf('Running avr_closedloop_2dof (Phase 2)...\n'); fprintf(runfid,'Running avr_closedloop_2dof...\n');
        avr_closedloop_2dof();
        fprintf(runfid,'avr_closedloop_2dof completed\n');
        % comparison plot
        if exist('avr_compare_controllers.m','file')
            fprintf('Running avr_compare_controllers...\n'); fprintf(runfid,'Running avr_compare_controllers...\n');
            avr_compare_controllers();
            fprintf(runfid,'avr_compare_controllers completed\n');
        end
    else
        fprintf(runfid,'avr_closedloop_2dof not found - skipping Phase 2\n');
    end
catch ME
    fprintf(runfid,'Phase2 failed: %s\n', ME.message);
end

% Optional Phase3 tuning
try
    if exist('avr_phase3_tune.m','file')
        fprintf('Running avr_phase3_tune (optional) ...\n'); fprintf(runfid,'Running avr_phase3_tune...\n');
        avr_phase3_tune();
        fprintf(runfid,'avr_phase3_tune completed\n');
    end
catch ME
    fprintf(runfid,'Phase3 tuning failed: %s\n', ME.message);
end

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
    % Ensure Phase 2 artifacts exist; auto-run Phase 2 if missing so Phase 3 has controllers
    phase2mat = fullfile(phase_artifacts('phase2').mat, 'avr_phase2.mat');
    if ~exist(phase2mat,'file')
        fprintf(runfid, 'Phase 2 artifacts missing (%s). Auto-running avr_closedloop_2dof to generate them...\n', phase2mat);
        try
            avr_closedloop_2dof();
            fprintf(runfid, 'Auto-ran avr_closedloop_2dof successfully.\n');
        catch ME2
            fprintf(runfid, 'Auto-run of Phase 2 failed: %s\n', ME2.message);
        end
    end
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
    p4dir = phase_artifacts('phase4').root;
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
csvpath = fullfile(phase_artifacts('phase5').csv, 'phase5_comparison.csv');
if exist(csvpath,'file')
    T = readtable(csvpath);
    outfig = fullfile(outRoot,'phase5_ITAE.png');
    try
        hf = figure('Visible','off');
        bar([T.itae_2dof, T.itae_pid, T.itae_res]);
        set(gca,'XTickLabel', cellstr(string(T.scenario_name)));
        legend('2DoF','PID','Resilient','Location','northwest');
        title('Phase5 ITAE Comparison'); ylabel('ITAE'); grid on;
        exportgraphics(hf, outfig, 'Resolution', 150);
        close(hf);
        fprintf('Saved Phase5 ITAE plot to %s\n', outfig);
    catch ME
        warning('Could not create phase5 ITAE plot: %s', ME.message);
    end
end

fprintf('All done. Logs and artifacts in results/run_all and results/phase5 if available.\n');
