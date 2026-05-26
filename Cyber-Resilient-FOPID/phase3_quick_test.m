% phase3_quick_test.m
% Quick Phase-3 smoke test: baseline -> attack injection -> detection

try
    % Ensure workspace functions are visible
    addpath(pwd);

    % Prepare results folder and log
    outdir = fullfile('results','phase3'); if ~exist(outdir,'dir'), mkdir(outdir); end
    run_ts = datestr(now,'yyyymmdd_HHMMSS');
    logpath = fullfile(outdir, ['phase3_quick_run_' run_ts '.log']);
    lf = fopen(logpath,'w');
    closeLog = lf > 2;
    if lf < 0
        warning('Could not open %s for writing; logging to console only.', logpath);
        lf = 1;
        closeLog = false;
    end
    fprintf(lf,'Phase3 quick run log - %s\n', datestr(now));
    fprintf('Phase3 quick log: %s\n', logpath);

    % Load parameters and build plant
    avr_parameters;
    G_amp = tf(Ka, [Ta 1]);
    G_exc = tf(Ke, [Te 1]);
    G_gen = tf(Kg, [Tg 1]);
    G_sen = tf(Ks, [Ts 1]);
    G_fwd = G_amp * G_exc * G_gen;

    % Time and reference
    Tfinal = 25;
    t = 0:0.01:Tfinal;
    r_ref = ones(size(t));

    % Quick PID baseline controller (fast to compute)
    try
        Cpid = pidtune(G_fwd * G_sen, 'PID');
    catch
        Cpid = pid(1, 1, 0.1);
        warning('pidtune failed — using fallback PID');
    end

    % Closed-loop transfer (reference -> output)
    % Use a simple 1-DOF approximation for quick test
    G_cl = minreal((G_fwd * Cpid) / (1 + G_fwd * Cpid * G_sen), 1e-6);

    % Baseline response
    y_true = lsim(G_cl, r_ref, t);

    % Define an attack
    attack_config.enabled = true;
    attack_config.type = 'bias';
    attack_config.magnitude = 0.5;    % adjust magnitude
    attack_config.start_time = 5.0;   % seconds

    % Inject attack
    y_meas = avr_attack_injector(y_true, t, attack_config);

    % Detector config (defaults are OK)
    detector_config = struct();
    detector_config.baseline_window = 6;   % seconds
    detector_config.window_size = 200;     % samples
    detector_config.threshold_factor = 5;  % more conservative
    detector_config.min_consecutive = 7;
    detector_config.startup_suppress = 6;

    % Run detector
    try
        [attack_flag, confidence, detection_time, residuals] = avr_detector(y_meas, t, G_fwd, r_ref, detector_config);
    catch ME
        fprintf(lf, 'Detector ERROR: %s\n', ME.message);
        rethrow(ME);
    end
    fprintf(lf, 'Detector: flag=%d, confidence=%g, detection_time=%s\n', double(attack_flag), confidence, num2str(detection_time));

    % Save results
    results.attack_flag = attack_flag;
    results.confidence = confidence;
    results.detection_time = detection_time;
    results.residuals = residuals;
    results.t = t;
    results.y_true = y_true;
    results.y_meas = y_meas;
    results.attack_config = attack_config;
    results.detector_config = detector_config;
    save('results_phase3_quick.mat', 'results');
    copyfile('results_phase3_quick.mat', fullfile(outdir, ['results_phase3_quick_' run_ts '.mat']));
    fprintf(lf, 'Saved results MAT to %s\n', fullfile(outdir, ['results_phase3_quick_' run_ts '.mat']));

    % Plots
    figure('Name','Phase3 Quick Test');
    subplot(3,1,1);
    plot(t, r_ref, '--k', 'LineWidth', 1); hold on;
    plot(t, y_true, 'b', 'LineWidth', 1);
    plot(t, y_meas, 'r', 'LineWidth', 1);
    legend('r_{ref}','y_{true}','y_{meas}');
    title('Reference and Outputs'); grid on;

    subplot(3,1,2);
    plot(t, residuals, 'k'); hold on;
    if ~isnan(detection_time)
        xline(detection_time, 'r--', 'LineWidth', 1.5);
    end
    ylabel('Residual'); grid on; title('Residuals and detection');

    subplot(3,1,3);
    Jk = abs(residuals) + movmean(abs(residuals), detector_config.window_size);
    plot(t, Jk, 'm'); hold on;
    % baseline threshold estimate
    idx_baseline_end = find(t <= detector_config.baseline_window, 1, 'last');
    sigma = std(residuals(1:idx_baseline_end));
    threshold = detector_config.threshold_factor * max(sigma, 1e-6);
    yline(threshold, 'r--', 'LineWidth', 1.5);
    legend('J_k','threshold'); grid on; title('Detection metric');

    fprintf('\n--- Quick test results ---\n');
    fprintf('Attack flag: %d\n', attack_flag);
    fprintf('Confidence: %.4f\n', confidence);
    if ~isnan(detection_time)
        fprintf('Detection time: %.3f s\n', detection_time);
        fprintf('Detection delay: %.3f s\n', detection_time - attack_config.start_time);
    else
        fprintf('Detection time: NaN (no detection)\n');
    end

catch ME
    fprintf('Error during quick test: %s\n', ME.message);
    rethrow(ME);
end
if exist('lf','var') && closeLog, fprintf(lf,'Run complete\n'); fclose(lf); end
