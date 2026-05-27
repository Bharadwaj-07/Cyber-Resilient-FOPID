% phase3_full_run.m
% Comprehensive Phase 3 test runner
% - Loads parameters and controllers (prefers avr_phase2.mat)
% - Runs baseline responses
% - Executes multiple attack scenarios
% - Runs detector and switcher for each scenario
% - Logs results to CSV and MAT, produces summary plots

timestamp = datestr(now,'yyyymmdd_HHMMSS');
logdir = fullfile(pwd, 'phase3_results');
if ~exist(logdir,'dir'), mkdir(logdir); end
global AVR_SHARED_LOG_FID AVR_SHARED_LOG_PATH
if exist('AVR_SHARED_LOG_FID','var') && ~isempty(AVR_SHARED_LOG_FID) && AVR_SHARED_LOG_FID > 0
    fidlog = AVR_SHARED_LOG_FID;
    closeLog = false;
    logfile = AVR_SHARED_LOG_PATH;
else
    logfile = fullfile(logdir, ['phase3_run_' timestamp '.log']);
    fidlog = fopen(logfile,'w');
    closeLog = fidlog > 2;
    if fidlog < 0, error('Cannot open log file'); end
end
csvfile = fullfile(logdir, ['phase3_summary_' timestamp '.csv']);
matfile = fullfile(logdir, ['phase3_data_' timestamp '.mat']);
fprintf(fidlog, 'Phase3 full run log — %s\n', datestr(now));

try
    % --- Setup ---
    fprintf(fidlog, '\nLoading parameters...\n');
    avr_parameters;
    G_amp = tf(Ka, [Ta 1]);
    G_exc = tf(Ke, [Te 1]);
    G_gen = tf(Kg, [Tg 1]);
    G_sen = tf(Ks, [Ts 1]);
    G_fwd = G_amp * G_exc * G_gen;

    % Try to load phase2 controllers if present
    if exist('avr_phase2.mat','file')
        fprintf(fidlog, 'Loading avr_phase2.mat controllers...\n');
        data = load('avr_phase2.mat');
        if isfield(data,'C_y')
            C_2dof_y = data.C_y;
        elseif isfield(data,'C_y_2dof')
            C_2dof_y = data.C_y_2dof;
        else
            C_2dof_y = [];
        end
        if isfield(data,'C_r')
            C_2dof_r = data.C_r;
        elseif isfield(data,'C_r_2dof')
            C_2dof_r = data.C_r_2dof;
        else
            C_2dof_r = C_2dof_y;
        end
        if isfield(data,'C_y_1dof')
            C_pid = data.C_y_1dof;
        else
            try
                C_pid = pidtune(G_fwd * G_sen, 'PID');
            catch
                C_pid = pid(1,1,0.1);
                warning('Using fallback PID');
            end
        end
    else
        fprintf(fidlog, 'avr_phase2.mat not found — using quick PID fallback.\n');
        try
            C_pid = pidtune(G_fwd * G_sen, 'PID');
        catch
            C_pid = pid(1,1,0.1);
            warning('Using fallback PID');
        end
        C_2dof_y = C_pid; % fallback: treat PID as 2DoF feedback path
        C_2dof_r = C_pid;
    end

    % Time base and reference
    Tfinal = 25;
    dt = 0.01;
    t = (0:dt:Tfinal)';
    r_ref = ones(size(t));

    % Baseline closed-loop response using DC or linear sim
    fprintf(fidlog, 'Computing baseline response...\n');
    try
        % Use C_pid as controller on feedback path for baseline
        G_cl = minreal((G_fwd * C_pid) / (1 + G_fwd * C_pid * G_sen), 1e-6);
        y_true = lsim(G_cl, r_ref, t);
    catch ME
        warning('Baseline sim failed: %s — using zeros', ME.message);
        y_true = zeros(size(t));
    end

    % Detector default config
    detector_config = struct();
    detector_config.baseline_window = 5;     % seconds
    detector_config.window_size = 100;       % samples
    detector_config.threshold_factor = 5;    % conservative
    detector_config.min_consecutive = 7;    % consecutive samples
    detector_config.startup_suppress = 6;

    % Switcher config
    switcher_config = struct();
    switcher_config.hysteresis_time = 2.0;
    switcher_config.recovery_time = 0.5;
    switcher_config.initial_mode = 1;

    % Define attack scenarios
    scenarios = {};
    sid = 0;
    % bias small
    sid = sid + 1; scenarios{sid} = struct('id',sid,'type','bias','magnitude',0.2,'start_time',6);
    % bias large
    sid = sid + 1; scenarios{sid} = struct('id',sid,'type','bias','magnitude',0.6,'start_time',6);
    % ramp
    sid = sid + 1; scenarios{sid} = struct('id',sid,'type','ramp','slope',0.05,'start_time',8);
    % sine
    sid = sid + 1; scenarios{sid} = struct('id',sid,'type','sine','magnitude',0.3,'frequency',1.0,'start_time',10);
    % late small
    sid = sid + 1; scenarios{sid} = struct('id',sid,'type','bias','magnitude',0.3,'start_time',12);

    % Prepare CSV
    csvfid = fopen(csvfile,'w');
    fprintf(csvfid,'id,type,magnitude,slope,frequency,start_time,detected,detection_time,detection_delay,confidence,mode_transitions,final_mode\n');

    results = struct(); results.scenarios = scenarios; results.runs = {};

    % Run scenarios
    for s = 1:length(scenarios)
        sc = scenarios{s};
        fprintf(fidlog,'\n--- Running scenario %d (%s) ---\n', sc.id, sc.type);
        % Build attack_config
        attack_config = struct(); attack_config.enabled = true;
        attack_config.type = sc.type;
        if isfield(sc,'magnitude'), attack_config.magnitude = sc.magnitude; else attack_config.magnitude = 0.3; end
        if isfield(sc,'slope'), attack_config.slope = sc.slope; end
        if isfield(sc,'frequency'), attack_config.frequency = sc.frequency; end
        attack_config.start_time = sc.start_time;

        % Create attacked measurement
        y_meas = avr_attack_injector(y_true, t, attack_config);

        % Detector run
        [attack_flag, confidence, detection_time, residuals] = avr_detector(y_meas, t, G_cl, r_ref, detector_config);
        detection_delay = NaN; if ~isnan(detection_time), detection_delay = detection_time - attack_config.start_time; end

        % Switcher run (simulate controller outputs and modes)
        try
            switcher_config.detector_attack_flag = attack_flag;
            switcher_config.detector_attack_time = detection_time;
            [u, mode_history, switch_times] = avr_switcher(y_meas, t, r_ref, C_2dof_r, C_2dof_y, C_pid, switcher_config);
        catch ME
            warning('Switcher failed: %s', ME.message);
            u = zeros(size(t)); mode_history = ones(size(t)); switch_times = [];
        end

        mode_transitions = size(switch_times,1);
        final_mode = mode_history(end);

        % Save run result
        runres = struct();
        runres.scenario = sc; runres.attack_config = attack_config; runres.attack_flag = attack_flag;
        runres.confidence = confidence; runres.detection_time = detection_time; runres.detection_delay = detection_delay;
        runres.mode_transitions = mode_transitions; runres.final_mode = final_mode; runres.switch_times = switch_times;
        runres.t = t; runres.y_true = y_true; runres.y_meas = y_meas; runres.residuals = residuals; runres.u = u; runres.mode_history = mode_history;

        results.runs{end+1} = runres;

        % Log to CSV
        fprintf(csvfid,'%d,%s,%.4f,%.4f,%.4f,%.3f,%d,%.4f,%.4f,%.6g,%d,%d\n', ...
            sc.id, sc.type, field_or_default(sc,'magnitude',NaN), field_or_default(sc,'slope',NaN), field_or_default(sc,'frequency',NaN), ...
            sc.start_time, double(attack_flag), NaN2num(detection_time), NaN2num(detection_delay), confidence, mode_transitions, final_mode);

        % Plot per-run figure
        hf = figure('Visible','off');
        subplot(3,1,1);
        plot(t, r_ref, '--k', t, y_true, 'b', t, y_meas, 'r'); legend('r','y_{true}','y_{meas}'); title(sprintf('Scenario %d: %s', sc.id, sc.type)); grid on;
        subplot(3,1,2);
        plot(t, residuals); hold on; if ~isnan(detection_time), xline(detection_time,'r--'); end; title('Residuals'); grid on;
        subplot(3,1,3);
        plot(t, mode_history); ylim([0.5 3.5]); title('Switcher mode history'); grid on;
        saveas(hf, fullfile(logdir, sprintf('scenario_%02d_%s.png', sc.id, sc.type)));
        close(hf);

        fprintf(fidlog, 'Scenario %d: detected=%d, detection_time=%.3f, delay=%.3f, mode_trans=%d, final_mode=%d\n', sc.id, attack_flag, NaN2num(detection_time), NaN2num(detection_delay), mode_transitions, final_mode);
    end

    % Save results
    save(matfile, 'results');
    fprintf(fidlog, '\nSaved results to %s and %s\n', matfile, csvfile);

catch ME
    fprintf(fidlog, 'ERROR: %s\n', ME.message);
    if exist('csvfid','var') && csvfid > 0, fclose(csvfid); end
    if closeLog && exist('fidlog','var') && fidlog > 0, fclose(fidlog); end
    rethrow(ME);
end

if exist('csvfid','var') && csvfid > 0, fclose(csvfid); end
if closeLog && exist('fidlog','var') && fidlog > 0, fclose(fidlog); end

fprintf('Phase 3 full run complete. Results in %s\n', logdir);

% Helper
function v = NaN2num(x)
    if isempty(x) || isnan(x), v = NaN; else v = x; end
end

function v = field_or_default(s, field, defaultValue)
    if isfield(s, field)
        v = s.(field);
    else
        v = defaultValue;
    end
end
