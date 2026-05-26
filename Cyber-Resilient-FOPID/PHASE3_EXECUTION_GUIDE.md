# Phase 3 Execution Guide

This guide documents the implemented workflow for the remaining phases in the roadmap. It is written for phase-wise verification and upload review.

## Scope

The repository now covers:

- Phase 1: AVR plant model
- Phase 2: 2DoF FOPID controller and tuning
- Phase 3: Attack injection and detection
- Phase 4: Resilient switching
- Phase 5: Validation matrix

## Files Involved

- [avr_attack_injector.m](avr_attack_injector.m)
- [avr_detector.m](avr_detector.m)
- [avr_switcher.m](avr_switcher.m)
- [avr_phase3_test.m](avr_phase3_test.m)
- [avr_phase3_tune.m](avr_phase3_tune.m)
- [avr_validation_matrix.m](avr_validation_matrix.m)
- [phase3_full_run.m](phase3_full_run.m)
- [avr_phase3_plot.m](avr_phase3_plot.m)
- [README_PHASE3.md](README_PHASE3.md)
- [PHASE3_ARCHITECTURE.md](PHASE3_ARCHITECTURE.md)

## Recommended Run Order

1. Phase 2 baseline
   - Run `avr_closedloop_2dof.m` first if you need to refresh `avr_phase2.mat`.
   - Expected result: stable step response, saved Phase 2 tuning artifacts.

2. Phase 3 smoke test
   - Run `phase3_quick_test.m`.
   - Expected result: baseline response is generated, a bias attack is injected after 5 s, and `avr_detector` raises `attack_flag = 1` after the attack begins.

3. Phase 3 tuning
   - Run `avr_phase3_tune.m`.
   - Expected result: a tuning grid is evaluated and `results/results_tune_detector.mat` is saved with a `best_cfg` recommendation.

4. Phase 3/4 integrated test
   - Run `avr_phase3_test.m`.
   - Expected result: bias, ramp, and sine attacks are injected one by one; detector output is logged; switcher transitions from 2DoF to PID after detection; per-scenario MAT and PNG files are saved in `results/`.

5. Full batch run
   - Run `phase3_full_run.m`.
   - Expected result: multiple scenarios are executed, logs are written to `phase3_results/`, CSV and MAT summaries are saved, and plots are generated for each scenario.

6. Phase 5 summary
   - Run `avr_validation_matrix.m`.
   - Expected result: a console table is printed with attack type, detection time, detection delay, switch counts, and ITAE metrics.

## Expected Results By Phase

### Phase 3: Attack Injection and Detection

- `avr_attack_injector.m`
  - Bias attack should create a constant offset after `start_time`.
  - Ramp attack should grow linearly from the start time.
  - Sine attack should oscillate with the configured amplitude and frequency.

- `avr_detector.m`
  - On a clean signal, `attack_flag` should remain `false` and `detection_time` should be `NaN`.
  - On attacked data, the detector should report a positive confidence and a finite detection time after the attack starts.
  - The result should be stable across repeated runs with the same data.

### Phase 4: Resilient Switching

- `avr_switcher.m`
  - The mode history should begin in normal mode.
  - After detection, the mode should transition to PID mode.
  - Switch timing should be logged without chatter.
  - If the signal quiets down, recovery mode can transition back to normal after the hysteresis delay.

### Phase 5: Validation Matrix

- `avr_validation_matrix.m`
  - The output table should list all scenario results.
  - Detection delay should be non-negative for valid detections.
  - False positives should remain zero for the clean pre-attack window.
  - Switched ITAE should stay bounded and should not explode compared with the baseline.

## Debugging And Tuning Knobs

Use these when the behavior needs improvement:

- `detector_config.baseline_window`
  - Increase this if the plant transient is still active when the threshold is computed.

- `detector_config.window_size`
  - Increase this to smooth residual spikes.
  - Decrease this if detection becomes too slow.

- `detector_config.threshold_factor`
  - Increase this to reduce false positives.
  - Decrease this to improve sensitivity for small attacks.

- `detector_config.min_consecutive`
  - Increase this if single-sample spikes still trigger detection.

- `detector_config.Q` and `detector_config.R`
  - Increase `Q` if the observer is too rigid.
  - Increase `R` if the observer is too trustful of measurements.

- `switcher_config.hysteresis_time`
  - Increase this if the mode chatters during recovery.

- `switcher_config.recovery_time`
  - Increase this if the system returns to normal too quickly.

## Notes On Current Implementation

- The detector uses a residual observer and a robust sliding metric.
- The switcher supports both heuristic triggering and detector-driven triggering.
- The validation script accepts both legacy result cell arrays and the newer structured result bundle.
- The full-run script writes its outputs into `phase3_results/` so the workspace stays organized.

## What To Check During Review

- Attack signals appear only after the configured start time.
- Detection occurs after the attack begins, not during the baseline interval.
- The switcher transitions to PID after detection and does not chatter.
- Result files and plots are created for each attack scenario.
- The validation matrix reports meaningful comparison metrics.
