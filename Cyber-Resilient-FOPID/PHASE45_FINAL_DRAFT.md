PHASE 4 & 5 FINAL DRAFT
=======================

Goal
----
Complete Phase 4 (Resilient switching with bumpless transfer) and Phase 5 (comprehensive validation matrix) so the repository contains:

- A robust detector (Phase 3)
- A resilient controller with bumpless transfer (Phase 4)
- A full comparison benchmark and artifacts (Phase 5)

Deliverables implemented in this pass
-------------------------------------
Files added/updated:

- avr_switcher.m (updated)
  - Continuous state-space simulation of both controllers
  - Detector-driven and metric-driven switching
  - Bumpless transfer by aligning the incoming controller internal state so output matches outgoing output
  - Mode history and switch_times logging

- avr_detector.m (updated)
  - Residual observer with median-based sliding metric
  - Baseline window, consecutive exceedances, min_consecutive parameter

- phase3_quick_test.m (added earlier)
- phase3_full_run.m (added earlier)
- phase5_full_comparison.m (new)
  - Runs scenarios and compares 2DoF, PID, and Resilient system
  - Generates per-scenario MAT and PNG files and a CSV summary in `results/phase5`

- PHASE3_EXECUTION_GUIDE.md (new)
- PHASE45_FINAL_DRAFT.md (this document)

Design and Implementation Details
---------------------------------
1) Bumpless transfer (Phase 4)

- Approach: simulate both controllers' state-space realizations in continuous time using Euler integration at the simulation time base `t`.
- At switch instant (either detector-driven or metric heuristic), we adjust the incoming controller's state `x` to satisfy:

  y_incoming = C_incoming * x_incoming + D_incoming * e(k)

  Solve for `x_incoming` with a pseudoinverse `pinv(C_incoming)` to set `x_incoming = pinv(C) * (u_prev - D*e)` where `u_prev` is the outgoing controller's most recent output.

- This is a practical bumpless transfer; it assumes the controller output depends on internal states linearly (state-space form). If `C` is non-invertible we skip adjustment.

- Controllers are updated every step; mode decides which output is applied to the plant simulation (the resilient approach updates both internal states but applies only active mode output).

2) Detector integration

- The detector computes residuals using a steady-state Kalman-like observer (LQE) on the nominal closed-loop baseline model.
- Baseline sigma is computed over `baseline_window` seconds. Threshold = `threshold_factor * sigma`.
- Detection metric uses `Jk = |e_k| + median(|residuals_recent|)` to reduce outlier sensitivity.
- Detection requires `min_consecutive` consecutive windows above threshold before firing.
- `avr_switcher` accepts detector hint via `switcher_config.detector_attack_flag` and `switcher_config.detector_attack_time` — when provided, it forces a switch at the detection time and then locks the run into attack handling to avoid repeated re-triggering on the same event.

3) Phase 5 comparison

- `phase5_full_comparison.m` runs several attack scenarios (bias small/large, ramp, sine) and records:
  - detection_time, detection_delay, confidence
  - ITAE for 2DoF, PID, and Resilient
  - mode transitions and final mode for resilient run
- Outputs: per-scenario `.mat` and `.png` in `results/phase5` and `phase5_comparison.csv` summarizing results.

Expected Results
----------------
For each scenario in Phase 5 expectations:

- 2DoF (no resilience): good nominal performance but may show large steady-state error or instability under some attacks.
- PID (classical): robust to attacks; stable but worse ITAE compared to tuned 2DoF in clean conditions.
- Resilient: detection within a small lag (configurable), switch to PID, degraded performance compared to 2DoF but remains stable and recovers.

Key metrics acceptance:
- Detection delay must be >= 0 and ideally <= 2s for listed scenarios.
- False positives (detections before attack start) should be zero in baseline window.
- Mode chatter should be negligible: mode_transitions per scenario <= 3 typically.

Testing & Debugging Recommendations
-----------------------------------
- If detection fires too early: increase `baseline_window` and/or `min_consecutive`, or raise `threshold_factor`.
- If detection misses attacks: reduce `threshold_factor`, increase `Q`, or reduce `R` to make observer more sensitive.
- If switching causes output spikes: check `pinv(C)` behavior; if `C` is rank-deficient consider regularized solve: `x = (C'*C + eps*I) \ (C'*(u_prev - D*e))`.
- If lsim of G_fwd with control sequence `u` produces unrealistic results, implement a proper closed-loop step-by-step integration where plant state is advanced using plant state-space matrices and controller output applied at each step.

What remains / Next steps
-------------------------
- Improve bumpless transfer: implement a regularized least-squares when `C` is non-invertible.
- Implement a full time-domain closed-loop integrator (plant + controller) rather than approximating plant response via `lsim(G_fwd, u, t)` for attacked runs.
- Add unit tests for `avr_detector` (false positive rate on noisy baseline) and `avr_switcher` (no control spikes across synthetic switch events).
- Provide a small Simulink model for high-fidelity closed-loop validation (optional).

Final notes for upload
----------------------
- Include `results/phase5` artifacts for reproducibility.
- Document in PR description the key parameters used (detector_config, switcher_config) and note if `avr_phase2.mat` was used or if controller fallback was applied.


Contact
-------
If you want, I can now:
- (A) Implement regularized state alignment for tougher controllers,
- (B) Replace `lsim`-based plant approximation with stepwise state-space integration for true closed-loop attacked simulation,
- (C) Run the Phase 5 comparison locally (requires MATLAB runtime) and attach sample results.

Which next step do you want me to do? (A/B/C)