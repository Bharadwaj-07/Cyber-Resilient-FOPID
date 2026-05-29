% run_all_phases.m
% Master runner: executes the full roadmap chain end-to-end.
% Phase 1 -> Phase 2 -> Phase 3 tuning/test -> Phase 5 comparison.
% Phase 4 is exercised through avr_switcher during Phase 5.
% Writes results to results/run_all/

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

run_summary = {};

% Phase 1: validate plant
try
    if exist('avr_validate_plant.m','file')
        fprintf('Running avr_validate_plant (Phase 1)...\n'); fprintf(runfid,'Running avr_validate_plant...\n');
        avr_validate_plant();
        fprintf(runfid,'avr_validate_plant completed\n');
        run_summary(end+1,:) = {'phase1','avr_validate_plant','ok'};
    else
        fprintf(runfid,'avr_validate_plant not found - skipping Phase 1\n');
        run_summary(end+1,:) = {'phase1','avr_validate_plant','missing'};
    end
catch ME
    fprintf(runfid,'Phase1 failed: %s\n', ME.message);
    run_summary(end+1,:) = {'phase1','avr_validate_plant','failed'};
end

% Phase 2: tuning and controller comparison
try
    phase2mat = fullfile(phase_artifacts('phase2').mat, 'avr_phase2.mat');
    if exist(phase2mat,'file')
        fprintf('Phase 2 artifacts found (%s) — skipping avr_closedloop_2dof (PSO).\n', phase2mat);
        fprintf(runfid,'Phase2 artifacts found (%s) — skipping avr_closedloop_2dof (PSO).\n', phase2mat);
        run_summary(end+1,:) = {'phase2','avr_closedloop_2dof','skipped'};
        % still run comparison plot if available
        if exist('avr_compare_controllers.m','file')
            try
                fprintf('Running avr_compare_controllers...\n'); fprintf(runfid,'Running avr_compare_controllers...\n');
                avr_compare_controllers();
                fprintf(runfid,'avr_compare_controllers completed\n');
            catch MEc
                fprintf(runfid,'avr_compare_controllers failed: %s\n', MEc.message);
            end
        end
    else
        if exist('avr_closedloop_2dof.m','file')
            fprintf('Running avr_closedloop_2dof (Phase 2)...\n'); fprintf(runfid,'Running avr_closedloop_2dof...\n');
            avr_closedloop_2dof();
            fprintf(runfid,'avr_closedloop_2dof completed\n');
            run_summary(end+1,:) = {'phase2','avr_closedloop_2dof','ok'};
            if exist('avr_compare_controllers.m','file')
                fprintf('Running avr_compare_controllers...\n'); fprintf(runfid,'Running avr_compare_controllers...\n');
                avr_compare_controllers();
                fprintf(runfid,'avr_compare_controllers completed\n');
            end
        else
            fprintf(runfid,'avr_closedloop_2dof not found - skipping Phase 2\n');
            run_summary(end+1,:) = {'phase2','avr_closedloop_2dof','missing'};
        end
    end
catch ME
    fprintf(runfid,'Phase2 failed: %s\n', ME.message);
    run_summary(end+1,:) = {'phase2','avr_closedloop_2dof','failed'};
end

% Phase 3 tuning (detector/switcher calibration)
try
    if exist('avr_phase3_tune.m','file')
        fprintf('Running avr_phase3_tune (Phase 3 tune)...\n'); fprintf(runfid,'Running avr_phase3_tune...\n');
        avr_phase3_tune();
        fprintf(runfid,'avr_phase3_tune completed\n');
        run_summary(end+1,:) = {'phase3_tune','avr_phase3_tune','ok'};
    end
catch ME
    fprintf(runfid,'Phase3 tuning failed: %s\n', ME.message);
    run_summary(end+1,:) = {'phase3_tune','avr_phase3_tune','failed'};
end

% Phase 3 quick smoke test
try
    if exist('phase3_quick_test.m','file')
        fprintf('Running phase3_quick_test...\n'); fprintf(runfid,'Running phase3_quick_test...\n');
        phase3_quick_test();
        fprintf(runfid,'phase3_quick_test completed\n');
        run_summary(end+1,:) = {'phase3_quick_test','phase3_quick_test','ok'};
    else
        fprintf(runfid,'phase3_quick_test not found - skipping\n'); fprintf(runfid,'phase3_quick_test not found - skipping\n');
        run_summary(end+1,:) = {'phase3_quick_test','phase3_quick_test','missing'};
    end
catch ME
    fprintf('Phase3 quick test failed: %s\n', ME.message);
    fprintf(runfid,'Phase3 quick test failed: %s\n', ME.message);
    run_summary(end+1,:) = {'phase3_quick_test','phase3_quick_test','failed'};
end

% Phase 3 full validation run
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
        run_summary(end+1,:) = {'phase3_full_run','phase3_full_run','ok'};
    else
        fprintf('phase3_full_run not found - skipping\n'); fprintf(runfid,'phase3_full_run not found - skipping\n');
        run_summary(end+1,:) = {'phase3_full_run','phase3_full_run','missing'};
    end
catch ME
    fprintf('Phase3 full run failed: %s\n', ME.message); fprintf(runfid,'Phase3 full run failed: %s\n', ME.message);
    run_summary(end+1,:) = {'phase3_full_run','phase3_full_run','failed'};
end

% Phase 5: full comparison and resilient validation (Phase 4 is exercised here)
try
    if exist('phase5_full_comparison.m','file')
        % Run quick 2-DoF/resilient diagnostic before Phase5
        if exist('phase5_check_2dof.m','file')
            try
                fprintf('Running phase5_check_2dof diagnostic...')
                fprintf(runfid,']Running phase5_check_2dof diagnostic...\n');
                phase5_check_2dof();
                fprintf(runfid,'phase5_check_2dof completed\n');
                run_summary(end+1,:) = {'phase5_check_2dof','phase5_check_2dof','ok'};
            catch MEchk
                fprintf(runfid,'phase5_check_2dof failed: %s\n', MEchk.message);
                run_summary(end+1,:) = {'phase5_check_2dof','phase5_check_2dof','failed'};
            end
        else
            fprintf(runfid,'phase5_check_2dof not found - skipping 2DoF diagnostic\n');
        end
        % Optional: run grid-search for recovery parameters first if available
        if exist('phase5_grid_search_recovery.m','file')
            try
                fprintf('Running phase5_grid_search_recovery (grid search)...\n'); fprintf(runfid,'Running phase5_grid_search_recovery...\n');
                phase5_grid_search_recovery();
                fprintf(runfid,'phase5_grid_search_recovery completed\n');
                run_summary(end+1,:) = {'phase5_grid_search_recovery','phase5_grid_search_recovery','ok'};
                % After grid completes, pick best params automatically
                if exist('phase5_apply_best_from_grid.m','file')
                    try
                        fprintf('Selecting best Phase5 config from grid results...\n'); fprintf(runfid,'Selecting best Phase5 config from grid results...\n');
                        phase5_apply_best_from_grid();
                        fprintf(runfid,'phase5 config selection completed\n');
                        run_summary(end+1,:) = {'phase5_apply_best_from_grid','phase5_apply_best_from_grid','ok'};
                    catch MEsel
                        fprintf(runfid,'phase5_apply_best_from_grid failed: %s\n', MEsel.message);
                        run_summary(end+1,:) = {'phase5_apply_best_from_grid','phase5_apply_best_from_grid','failed'};
                    end
                else
                    fprintf(runfid,'phase5_apply_best_from_grid not found - skipping auto-selection\n');
                end
            catch MEgs
                fprintf(runfid,'phase5_grid_search_recovery failed: %s\n', MEgs.message);
                run_summary(end+1,:) = {'phase5_grid_search_recovery','phase5_grid_search_recovery','failed'};
            end
        else
            fprintf(runfid,'phase5_grid_search_recovery not found - skipping grid search\n');
        end

        % Run the top-3 evaluator if available. This gives a deeper per-scenario
        % comparison of the candidate configs before the final comparison pass.
        if exist('phase5_evaluate_top3_from_grid.m','file')
            try
                fprintf('Running phase5_evaluate_top3_from_grid...\n'); fprintf(runfid,'Running phase5_evaluate_top3_from_grid...\n');
                phase5_evaluate_top3_from_grid();
                fprintf(runfid,'phase5_evaluate_top3_from_grid completed\n');
                run_summary(end+1,:) = {'phase5_evaluate_top3_from_grid','phase5_evaluate_top3_from_grid','ok'};
            catch MEeval
                fprintf(runfid,'phase5_evaluate_top3_from_grid failed: %s\n', MEeval.message);
                run_summary(end+1,:) = {'phase5_evaluate_top3_from_grid','phase5_evaluate_top3_from_grid','failed'};
            end
        else
            fprintf(runfid,'phase5_evaluate_top3_from_grid not found - skipping top-3 evaluation\n');
        end

        % Run the gap-vs-continuous assessment so the pipeline captures the
        % repeated-attack-with-recovery-gaps scenario separately.
        if exist('phase5_assess_gap_attacks.m','file')
            try
                fprintf('Running phase5_assess_gap_attacks...\n'); fprintf(runfid,'Running phase5_assess_gap_attacks...\n');
                phase5_assess_gap_attacks();
                fprintf(runfid,'phase5_assess_gap_attacks completed\n');
                run_summary(end+1,:) = {'phase5_assess_gap_attacks','phase5_assess_gap_attacks','ok'};
            catch MEgap
                fprintf(runfid,'phase5_assess_gap_attacks failed: %s\n', MEgap.message);
                run_summary(end+1,:) = {'phase5_assess_gap_attacks','phase5_assess_gap_attacks','failed'};
            end
        else
            fprintf(runfid,'phase5_assess_gap_attacks not found - skipping gap attack assessment\n');
        end

        fprintf('Running phase5_full_comparison...\n'); fprintf(runfid,'Running phase5_full_comparison...\n');
        phase5_full_comparison();
        fprintf(runfid,'phase5_full_comparison completed\n'); fprintf(runfid,'phase5_full_comparison completed\n');
        run_summary(end+1,:) = {'phase5_full_comparison','phase5_full_comparison','ok'};
        % Run additional Phase5 comparison scripts if present
        if exist('tools/phase5_multi_setup_compare.m','file')
            try
                fprintf('Running tools/phase5_multi_setup_compare...\n'); fprintf(runfid,'Running tools/phase5_multi_setup_compare...\n');
                tools/phase5_multi_setup_compare();
                fprintf(runfid,'phase5_multi_setup_compare completed\n');
                run_summary(end+1,:) = {'phase5_multi_setup_compare','tools/phase5_multi_setup_compare','ok'};
            catch MEm
                fprintf(runfid,'phase5_multi_setup_compare failed: %s\n', MEm.message);
                run_summary(end+1,:) = {'phase5_multi_setup_compare','tools/phase5_multi_setup_compare','failed'};
            end
        else
            fprintf(runfid,'phase5_multi_setup_compare not found - skipping\n');
        end
        if exist('tools/phase5_augmented_compare.m','file')
            try
                fprintf('Running tools/phase5_augmented_compare...\n'); fprintf(runfid,'Running tools/phase5_augmented_compare...\n');
                tools/phase5_augmented_compare();
                fprintf(runfid,'phase5_augmented_compare completed\n');
                run_summary(end+1,:) = {'phase5_augmented_compare','tools/phase5_augmented_compare','ok'};
            catch MEnt
                fprintf(runfid,'phase5_augmented_compare failed: %s\n', MEnt.message);
                run_summary(end+1,:) = {'phase5_augmented_compare','tools/phase5_augmented_compare','failed'};
            end
        else
            fprintf(runfid,'phase5_augmented_compare not found - skipping\n');
        end
    else
        fprintf('phase5_full_comparison not found - skipping\n'); fprintf(runfid,'phase5_full_comparison not found - skipping\n');
        run_summary(end+1,:) = {'phase5_full_comparison','phase5_full_comparison','missing'};
    end
catch ME
    fprintf('Phase5 full comparison failed: %s\n', ME.message); fprintf(runfid,'Phase5 full comparison failed: %s\n', ME.message);
    run_summary(end+1,:) = {'phase5_full_comparison','phase5_full_comparison','failed'};
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
        % If Phase5 shows 2DoF catastrophically worse than PID in any scenario,
        % trigger a robust re-tuning of Phase2 using a robustness penalty.
        bad_idx = find(~isfinite(T.itae_2dof) | (T.itae_2dof > 10 * T.itae_pid));
        if ~isempty(bad_idx)
            fprintf(runfid, 'Phase5 indicates 2DoF fragile in %d scenarios — triggering robust Phase2 re-tune.\n', numel(bad_idx));
            try
                % Re-run Phase2 with robust_mode enabled
                opts = struct(); opts.robust_mode = 'sensitivity'; opts.robust_weight = 100; opts.n_particles = 40; opts.max_iter = 150;
                fprintf(runfid, 'Starting robust Phase2 tuning...\n');
                robust_opts = opts; avr_closedloop_2dof(robust_opts);
                fprintf(runfid, 'Robust Phase2 tuning complete. Re-running Phase3 and Phase5...\n');
                phase3_full_run();
                phase5_full_comparison();
            catch ME
                fprintf(runfid, 'Robust re-tune failed: %s\n', ME.message);
            end
        end
    catch ME
        warning('Could not create phase5 ITAE plot: %s', ME.message);
    end
end

try
    if ~isempty(run_summary)
        Tsum = cell2table(run_summary, 'VariableNames', {'phase','script_name','status'});
        writetable(Tsum, fullfile(outRoot, 'run_all_summary.csv'));
        save(fullfile(outRoot, 'run_all_summary.mat'), 'Tsum');
    end
catch ME
    warning('Could not write run_all summary: %s', ME.message);
end

fprintf('All done. Logs and artifacts in results/run_all and results/phase5 if available.\n');
