Phase 3: Attack Injection, Detection, and Resilient Switching

Files added:
- avr_attack_injector.m
- avr_detector.m
- avr_switcher.m
- avr_phase3_test.m
- avr_phase3_tune.m
- avr_validation_matrix.m
- phase3_full_run.m
- phase3_quick_test.m
- PHASE3_ARCHITECTURE.md
- PHASE3_EXECUTION_GUIDE.md

Quick run steps (in MATLAB):

1) Open MATLAB and set folder to the project root:

```matlab
cd('c:/Users/bhara/Downloads/Cyber-Resilient-FOPID/Cyber-Resilient-FOPID')
```

2) Run Phase 3 tests (uses tuned Phase 2 results in `avr_phase2.mat`):

```matlab
% Run the standard Phase 3 test suite
avr_phase3_test
% View consolidated results
avr_validation_matrix
```

3) Auto-tune detector parameters (sweep grid across multiple attack types):

```matlab
% Runs grid search evaluating bias, ramp, and sine attacks; saves results/results_tune_detector.mat
avr_phase3_tune
```

4) If `avr_phase3_tune` suggests a best config, use it by editing `avr_phase3_test.m` detector_cfg, or pass a custom detector config to `avr_detector`.

5) For a broader phase-wise batch run, use:

```matlab
phase3_full_run
```

What to watch for when running:

- False positives: detections before the attack start indicate threshold too low. Increase `threshold_factor` or widen `baseline_window`.
- Missed detections: no detection or very late detection => lower `threshold_factor`, increase `Q` (process noise), or reduce `R` (measurement noise) to make the filter trust measurements less/more accordingly.
- Baseline window selection: baseline should be after the transient settles (we use 5s). If your step response is slower, increase `baseline_window`.
- Sampling rate: scripts assume 1 kHz sampling (`t = 0:0.001:Tfinal`). If you change, adjust `window_size` accordingly (100 samples = 0.1s by default).
- Numerical issues in Kalman filter: if `lqe` fails due to model size, reduce `Q` and `R` or use a simple observer gain fallback.
- Switching chatter: increase `hysteresis_time` and `recovery_time` if the mode changes too often.
- Premature detection: increase `min_consecutive` or the baseline window if the detector is firing before the attack start time.

Expected outcomes when the implementation is healthy:

- Clean baseline runs produce `attack_flag = false`, `detection_time = NaN`, and low residuals.
- Bias, ramp, and sine attacks are injected only after `start_time` and are visible in `y_meas`.
- Detected attacks report a finite detection time after the attack starts, not during the baseline interval.
- The switcher transitions to PID after a valid detection and logs mode changes.
- Result files are written to `results/` and `phase3_results/` with plots and CSV/MAT summaries.

How to iterate tuning:

1. Run `avr_phase3_test` with default configs and inspect `results/*.mat` and the plotted residuals.
2. Run `avr_phase3_tune` to find a recommended detector config on the bias attack. The script saves `results/results_tune_detector.mat`.
3. Re-run `avr_phase3_test` with recommended detector parameters and inspect detection latency and false-positive rates for all attack types.
4. If false positives occur during transient, increase `baseline_window` to exclude transient region.
5. For production, consider implementing an adaptive threshold that scales with recent residual variance.

See `PHASE3_EXECUTION_GUIDE.md` for the full phase-wise run order, expected outputs, and debugging knobs.

Contact notes:
- The modular design keeps plant and controller code unchanged; tuning touches only detector parameters and optionally switcher thresholds.
- If you want, I can now run `avr_phase3_tune` locally (I cannot run MATLAB here) or prepare a parameter sweep with tighter ranges.

