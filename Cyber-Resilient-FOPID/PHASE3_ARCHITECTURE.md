# Phase 3: Attack Injection & Resilient Detection
## Architecture Design Document

**Status**: Design (ready for code implementation)  
**Version**: 1.0  
**Date**: May 26, 2026

---

## 1. Overview & Objectives

### Goal
Extend the tuned 2DoF FOPID controller with a **measurement attack layer** that:
1. Injects known attack signals (bias, ramp, sinusoid) to test resilience
2. Detects attacks using Kalman filter residuals
3. Switches to classical PID when attack is detected
4. Maintains stability throughout normal, attack, and recovery modes

### Key Constraint
**"System remains stable if attack is detected within t_d seconds"**

This is the core theoretical contribution: prove that detection latency t_d bounds the system's trajectory in LHP.

---

## 2. Attack Model Definition

### 2.1 Attack Signal Types

All attacks corrupt the **feedback measurement** path:
$$y_{\text{meas}}(t) = y_{\text{true}}(t) + a(t)$$

where $a(t)$ is the injected attack signal.

#### Type A: Constant Bias Attack
$$a(t) = \begin{cases} A_0 & \text{if } t \geq t_{\text{start}} \\ 0 & \text{otherwise} \end{cases}$$

**Parameters**:
- Magnitude: $A_0 = 0.1$ pu (typical ±10% deviation from nominal terminal voltage)
- Start time: $t_{\text{start}} = 5$ s (after transient settles)
- Duration: until end of simulation (10s)

**Interpretation**: Sensor malfunction (stuck reading) or actuator bias in exciter path.

---

#### Type B: Ramp Attack
$$a(t) = \begin{cases} m \cdot (t - t_{\text{start}}) & \text{if } t \geq t_{\text{start}} \\ 0 & \text{otherwise} \end{cases}$$

**Parameters**:
- Ramp slope: $m = 0.05$ pu/s
- Start time: $t_{\text{start}} = 5$ s
- Maximum attack (at $t=10$s): $0.05 \times 5 = 0.25$ pu

**Interpretation**: Slowly drifting sensor bias (common in aging equipment).

---

#### Type C: Sinusoidal Attack
$$a(t) = \begin{cases} A \sin(2\pi f \cdot (t - t_{\text{start}})) & \text{if } t \geq t_{\text{start}} \\ 0 & \text{otherwise} \end{cases}$$

**Parameters**:
- Frequency: $f = 1$ Hz
- Amplitude: $A = 0.1$ pu
- Start time: $t_{\text{start}} = 5$ s

**Interpretation**: Oscillatory measurement noise (EMI, harmonic disturbance).

---

### 2.2 Attack Injection Module (`avr_attack_injector.m`)

**Responsibility**: Generate corrupted measurement signal

**Interface**:
```matlab
function y_meas = avr_attack_injector(y_true, t, attack_config)
    % Inputs:
    %   y_true (N×1):     True plant output from simulation
    %   t (N×1):          Time vector [0, 0.001, ..., 10]
    %   attack_config:    struct with fields:
    %     .enabled (bool):    Enable attack injection
    %     .type (char):       'bias' | 'ramp' | 'sine'
    %     .magnitude (float): A_0 or A (depending on type)
    %     .slope (float):     m (for ramp only)
    %     .frequency (float): f (for sine only)
    %     .start_time (float): t_start (default: 5 s)
    %
    % Output:
    %   y_meas (N×1):      Measurement with attack: y_true + a(t)
end
```

**Algorithm**:
```
1. Initialize: a = zeros(size(t))
2. Find indices: idx_start = find(t >= attack_config.start_time, 1)
3. Switch on attack_config.type:
   'bias':  a(idx_start:end) = attack_config.magnitude
   'ramp':  a(idx_start:end) = attack_config.slope * (t(idx_start:end) - t(idx_start))
   'sine':  a(idx_start:end) = attack_config.magnitude * sin(2π*attack_config.frequency*(t(idx_start:end)-t(idx_start)))
4. Return: y_meas = y_true + a
```

**Key Property**: Attack is added **after** true plant output is simulated. Attack does NOT affect plant dynamics, only measurement feedback.

---

## 3. Detection Logic

### 3.1 Kalman Filter Residual Detector

**Principle**: In the absence of attack, a well-tuned Kalman filter predicts the measurement accurately. When attack injects bias, the residual (innovation) $e(t) = y_{\text{meas}}(t) - \hat{y}(t)$ will show anomalous statistics (elevated mean, variance, or autocorrelation).

### 3.2 Kalman Filter Design

**Plant for estimation**: Use the **nominal plant without controller** (open-loop) as the observer basis.

Plant model:
$$\dot{x} = A_p x + B_p r$$
$$y = C_p x$$

where:
- $x$: Internal plant states (voltage, rotor angle, etc.)
- $r$: Reference input (step setpoint)
- $y$: Measured terminal voltage

**Kalman Filter**:
$$\hat{x}_{k|k-1} = A_p \hat{x}_{k-1} + B_p r_k$$
$$\hat{y}_{k|k-1} = C_p \hat{x}_{k|k-1}$$
$$\text{Residual: } e_k = y_{\text{meas},k} - \hat{y}_{k|k-1}$$

**Observer Gain**: Use steady-state Kalman gain $K_{\infty}$ computed from:
- Process noise covariance: $Q = 10^{-6} I$ (very small; plant is deterministic)
- Measurement noise covariance: $R = 10^{-4}$ (sensor noise ~0.01 pu RMS)

$$K_{\infty} = \text{solve steady-state Riccati equation}$$

### 3.3 Detection Algorithm

**Residual-based threshold**:

For detection at time $t_k$, compute a **normalized residual metric**:

$$J_k = |e_k| + \frac{1}{N_w} \sum_{i=k-N_w+1}^{k} |e_i|$$

where $N_w = 100$ is a sliding window (0.1 s at 1 kHz sampling).

**Detection Decision**:
$$\text{Attack detected if } J_k > \theta_{\text{res}}$$

**Threshold Selection**:
$$\theta_{\text{res}} = 2 \times \sigma_{\text{baseline}}$$

where $\sigma_{\text{baseline}}$ is the residual standard deviation during the **first 5 seconds** (pre-attack, steady-state).

**Detection Latency** ($t_d$):
$$t_d = t_{\text{first\_detection}} - t_{\text{attack\_start}}$$

---

### 3.4 Detector Module (`avr_detector.m`)

**Interface**:
```matlab
function [attack_flag, confidence, detection_time, residuals] = avr_detector(...
    y_meas, t, plant_model, r_ref, detector_config)
    % Inputs:
    %   y_meas (N×1):          Measured output
    %   t (N×1):               Time vector
    %   plant_model (struct):  avr_parameters + TF objects
    %   r_ref (N×1):           Reference input (setpoint)
    %   detector_config:       struct with:
    %     .baseline_window (float): duration (e.g., 5 s) for pre-attack baseline
    %     .detection_threshold (float): 2×σ (or user-specified)
    %     .window_size (int): sliding window for residual averaging
    %
    % Outputs:
    %   attack_flag (bool):    True if attack detected
    %   confidence (float):    Metric value at detection (J_k)
    %   detection_time (float): Time of first detection
    %   residuals (N×1):       Full residual history for plotting
end
```

**Algorithm**:
```
1. Build Kalman filter from nominal plant (open-loop, no controller)
2. For t ∈ [0, baseline_window]:
     Compute residuals, store σ_baseline = std(residuals)
     threshold = 2 × σ_baseline
3. Initialize: attack_flag = false, detection_time = nan
4. For k = 1:N (time steps):
     Compute Kalman prediction: ŷ_k|k-1 = C·x̂ + feedthrough
     Residual: e_k = y_meas(k) - ŷ_k|k-1
     Metric: J_k = |e_k| + (1/N_w)·sum(|e_i| for recent window)
     If J_k > threshold AND ~attack_flag:
         attack_flag = true
         detection_time = t(k)
         confidence = J_k
         break
5. Return all outputs
```

---

## 4. Resilient Switching Control

### 4.1 Control Architecture

**Three control modes**:

#### Mode 1: Normal (Attack-Free)
$$u(t) = -C_{\text{2DoF}}(s) \cdot [y_{\text{meas}}(t) - r(t)]$$

where $C_{\text{2DoF}}$ is the tuned 2DoF FOPID from Phase 2.

#### Mode 2: Attack Detected (Switching)
$$u(t) = -C_{\text{PID}}(s) \cdot [y_{\text{meas}}(t) - r(t)]$$

where $C_{\text{PID}}$ is the classical PID (Ziegler-Nichols or pidtune baseline).

**Rationale**: PID is proven stable in literature; switching to it guarantees stability even if detector is imperfect.

#### Mode 3: Recovery (Optional Hysteresis)
If attack is no longer detected for $t_{\text{hysteresis}} = 2$ s, consider switching back to 2DoF FOPID.

---

### 4.2 Switching Hysteresis Logic

**State Machine**:
```
State: {normal, attacking, recovery}
Transition guards:
  normal → attacking:   attack_flag = true (no delay)
  attacking → recovery: attack_flag = false (after 0.5 s)
  recovery → normal:    no attack detected for 2 s
  recovery → attacking: attack_flag = true (re-detected)
  recovery → attacking: back to PID if still unstable
```

**Bumpless Transfer** (to avoid input discontinuity):
$$u(t^+) = u(t^-) \text{ (maintain integral state)}$$

---

### 4.3 Switcher Module (`avr_switcher.m`)

**Interface**:
```matlab
function [u, mode_history, switch_times] = avr_switcher(...
    y_meas, t, r_ref, C_2dof, C_pid, attack_flag, attack_time, switcher_config)
    % Inputs:
    %   y_meas (N×1):         Measured output
    %   t (N×1):              Time vector
    %   r_ref (N×1):          Reference setpoint
    %   C_2dof:               2DoF FOPID controller (TF)
    %   C_pid:                Classical PID controller (TF)
    %   attack_flag (bool):   From detector
    %   attack_time (float):  From detector
    %   switcher_config:      struct with:
    %     .hysteresis_time (float): e.g., 2 s
    %     .recovery_threshold (float): e.g., 0.5 s
    %
    % Outputs:
    %   u (N×1):              Control signal
    %   mode_history (N×1):   [1=normal, 2=attacking, 3=recovery]
    %   switch_times (M×2):   [time, mode_from, mode_to] for each switch
end
```

---

## 5. Stability Analysis

### 5.1 Stability Claim

**Theorem**: *If an attack is detected within $t_d \leq t_{\max}$ seconds of its injection, the closed-loop system remains stable (poles in left-half plane) during and after detection.*

### 5.2 Proof Sketch

**Assumptions**:
1. Classical PID ($C_{\text{PID}}$) is designed to stabilize the AVR plant (verified in Phase 2 baseline).
2. Kalman filter residuals converge to detection within $t_d$ of attack start.
3. Switching is instantaneous and bumpless (integral state preserved).

**Proof by mode analysis**:

**Before attack** ($t < t_{\text{start}}$):
- Closed-loop: $T_1(s) = \frac{G(s) C_{\text{2DoF}}}{1 + G(s) C_{\text{2DoF}} H(s)}$
- Poles verified in Phase 2 tuning: all in LHP.

**During undetected attack** ($t_{\text{start}} < t < t_{\text{detection}}$):
- Measurement feedback is corrupted: $y_{\text{meas}} = y_{\text{true}} + a(t)$
- Reference error: $e(t) = r - y_{\text{meas}} = r - y_{\text{true}} - a(t)$
- Controller responds to inflated error and may saturate or become aggressive.
- **Risk**: 2DoF FOPID with fractional order can amplify high-frequency attack content.
- **Bounded assumption**: $|a(t)| \leq A_{\max} = 0.25$ pu (ramp at $t=10$s).
- **Finite Time Interval**: $t_d$ is bounded (detector fires within 1-2 seconds for our scenarios).

**At detection** ($t = t_{\text{detection}}$):
- Switch to PID: $u(t) = -C_{\text{PID}} (r - y_{\text{meas}})$
- PID is known stable (proven in Phase 2 and classical control literature).
- Even with corrupted measurement, PID's lower-order (3rd vs. 7th for FOPID) dynamics are more robust.

**After detection and recovery** ($t > t_{\text{detection}} + t_{\text{recovery}}$):
- If attack persists or is re-detected, remain in PID mode (guaranteed stable).
- If attack stops, Kalman residuals return to baseline, recovery hysteresis can re-enable 2DoF (but only after verified quiet period).

**Conclusion**: Stability is maintained because:
1. Bounded detection latency $t_d$ limits the interval of uncontrolled response.
2. Switch to PID provides a provably stable fallback.
3. Hysteresis prevents oscillation between modes.

---

### 5.3 Lyapunov Argument (Optional Formal Proof)

For rigorous publication, define:
$$V(x, u) = x^T P x + \int_0^t \|u(\tau)\|^2 d\tau$$

Show that:
- During normal mode: $\dot{V} < 0$ (closed-loop is exponentially stable)
- At switching: $V$ may increase (attack-induced energy), but remains bounded
- During PID mode: $\dot{V} < 0$ (PID recovers stability)
- Overall: $V(t=\infty) < V(t=0) + M \cdot A_{\max} \cdot t_d$ (bounded by attack magnitude and detection latency)

---

## 6. Validation Matrix

### 6.1 Test Scenarios

| Scenario | Attack Type | Controller | Expected Outcome |
|----------|-------------|------------|------------------|
| 1A | Constant Bias (0.1 pu) | 2DoF FOPID + Detector | Detected within 2s, switch to PID, stable |
| 1B | Constant Bias (0.1 pu) | PID (no detection) | Steady error, but stable |
| 1C | Constant Bias (0.1 pu) | 2DoF FOPID (no detector) | May oscillate; test robustness |
| 2A | Ramp (0.05 pu/s) | 2DoF FOPID + Detector | Detected within 1.5s, switch, stable |
| 2B | Ramp (0.05 pu/s) | PID (no detection) | Increasing error, but stable |
| 2C | Ramp (0.05 pu/s) | 2DoF FOPID (no detector) | Growing oscillation; test limit |
| 3A | Sinusoid (1 Hz, 0.1 pu) | 2DoF FOPID + Detector | Detected within 1s, switch, stable |
| 3B | Sinusoid (1 Hz, 0.1 pu) | PID (no detection) | Measurement jitter, but stable |
| 3C | Sinusoid (1 Hz, 0.1 pu) | 2DoF FOPID (no detector) | May resonate; test bandwidth |

### 6.2 Metrics to Compute

For each scenario, log and compare:

- **Detection Metrics**:
  - $t_d$: Detection latency (seconds)
  - Confidence: $J_k$ value at detection
  - False-positive rate: Detections before attack starts

- **Control Metrics**:
  - Rise time $T_r$ (s)
  - Settling time $T_s$ (s, 2% threshold)
  - Overshoot OS (%)
  - ITAE = $\int t |e(t)| dt$
  - Peak error during attack
  - Steady-state error (post-attack, if applicable)

- **Stability Metrics**:
  - All poles in LHP? (yes/no)
  - Magnitude of rightmost pole
  - Margin to instability (closest pole to RHP boundary)

---

## 7. File Structure & Dependencies

### File Organization (Modular)

```
Cyber-Resilient-FOPID/
├── avr_parameters.m                    [Phase 1 - unchanged]
├── avr_plant_model.m                   [Phase 1 - unchanged]
├── avr_validate_plant.m                [Phase 1 - unchanged]
├── fopid_operator.m                    [Phase 2 - unchanged]
├── fopid_2dof.m                        [Phase 2 - unchanged]
├── pso_tuner.m                         [Phase 2 - unchanged]
├── avr_closedloop_2dof.m               [Phase 2 - unchanged]
├── avr_compare_controllers.m           [Phase 2 - unchanged]
│
├── avr_attack_injector.m               [Phase 3A - NEW]
├── avr_detector.m                      [Phase 3B - NEW]
├── avr_switcher.m                      [Phase 3C - NEW]
│
├── avr_phase3_test.m                   [Phase 3 - Main runner]
├── avr_validation_matrix.m             [Phase 5 - Validation runner]
│
├── Roadmap.md                          [Documentation]
├── PHASE3_ARCHITECTURE.md              [This file]
└── results/                            [Output folder]
    ├── phase3_detection_*.mat
    ├── phase3_switching_*.mat
    ├── phase3_validation_*.mat
    └── phase3_*.png                    [Plots]
```

### Call Sequence

```
avr_phase3_test.m
├── Load avr_parameters.m
├── Load avr_phase2.mat (tuned 2DoF FOPID params)
├── Build plant TFs (G, H)
├── Build C_2dof from phase2 params
├── Build C_pid via zn_pid or pidtune
├── Load avr_baseline.mat (PID metrics)
│
├── LOOP: For each attack_type ∈ {bias, ramp, sine}
│   ├── Run closed-loop simulation: [y_true, u_2dof, t]
│   │   └── Uses C_2dof only (no attack, no detection yet)
│   │
│   ├── Generate attack: y_meas = y_true + a(t)
│   │   └── Call avr_attack_injector.m
│   │
│   ├── Detect attack: [attack_flag, conf, t_det, residuals] = avr_detector.m
│   │   └── Kalman filter on nominal plant
│   │
│   ├── Switch control: [u_switched, mode, switch_times] = avr_switcher.m
│   │   └── Re-simulate with switched control law
│   │
│   ├── Compute metrics: rise, settle, OS, ITAE, poles
│   │
│   └── Store results in cell array
│
└── Compile validation matrix and plot
    └── Call avr_validation_matrix.m or inline comparison
```

---

## 8. Implementation Checklist

- [ ] **avr_attack_injector.m**: Generate bias, ramp, sine attack signals
  - [ ] Parameter parsing (magnitude, slope, frequency, start_time)
  - [ ] Vectorized attack generation over time array
  - [ ] Unit tests: verify attack shape, magnitude

---

## Additions After Initial Architecture (implemented)

Since the initial architecture was written, the following items have been implemented and integrated into Phase 3 code and workflow:

- Multi-attack detector tuning: `avr_phase3_tune.m` performs a grid search and evaluates detector performance across three attack types (bias, ramp, sine). It also sweeps `threshold_factor`, `Q` and `R` scales, and switcher `hysteresis_time` and `recovery_time`, selecting the best aggregate configuration.
- Plotting utility: `avr_phase3_plot.m` generates and saves PNG visualizations for each scenario showing `y_true`, `y_meas`, residuals, detection time, control `u`, and mode history.
- Auto-apply best config: `avr_phase3_test.m` now loads `results/results_tune_detector.mat` (if present) and applies `best_cfg` to set `detector_cfg` and `switcher_cfg` automatically before running scenarios. The chosen config is saved into `results/phase3_all.mat`.
- Tuning-aware scoring: Tuning now scores configurations by detection latency, switched ITAE, and heavily penalizes misses and false positives to favor robust, timely detection.
- Result artifacts: Per-scenario `.mat` and `.png` files are saved in `results/` for quick inspection.

These additions are implemented to enable block-wise verification: we can now tune, run, visualize, and iterate per attack type and per module.
  
- [ ] **avr_detector.m**: Kalman filter residual detector
  - [ ] Build observer from nominal plant TFs
  - [ ] Compute baseline statistics (first 5 seconds)
  - [ ] Implement sliding-window residual metric
  - [ ] Detection threshold calculation
  - [ ] Return detection time and confidence
  - [ ] Unit tests: false-positive rate on clean signal, detection latency on attack

- [ ] **avr_switcher.m**: State machine and bumpless switching
  - [ ] Initialize state (normal)
  - [ ] Transition guards (hysteresis, recovery)
  - [ ] Dual-integrator tracking for integral state preservation
  - [ ] Switch between C_2dof and C_pid control laws
  - [ ] Log mode history and switch times
  - [ ] Unit tests: smooth transitions, no control spikes

- [ ] **avr_phase3_test.m**: Main Phase 3 test runner
  - [ ] Load all dependencies (parameters, phase2 results, controllers)
  - [ ] Loop over attack types
  - [ ] Run simulation → injector → detector → switcher
  - [ ] Compute metrics (ITAE, rise/settle, poles)
  - [ ] Save results to .mat files
  - [ ] Generate plots (y_true vs. y_meas, residuals, control signal, mode transitions)

- [ ] **avr_validation_matrix.m**: Consolidate results
  - [ ] Load all phase3 .mat files
  - [ ] Build comparison table [attack × controller]
  - [ ] Print to console and save as figure
  - [ ] Statistical summary (mean, std detection latency; average ITAE per mode)

- [ ] **Documentation**:
  - [ ] Code comments: equation numbers from PHASE3_ARCHITECTURE.md
  - [ ] Function headers with input/output specs
  - [ ] Example usage in each file

## 8. Implementation Status

The repository now contains the working Phase 3 to Phase 5 workflow:

- `avr_attack_injector.m` injects bias, ramp, and sinusoidal measurement attacks.
- `avr_detector.m` computes residuals with a baseline threshold and a consecutive-exceedance decision rule.
- `avr_switcher.m` supports both heuristic switching and detector-driven switching to PID.
- `avr_phase3_test.m` runs the attack, detect, switch, and metric capture flow per attack type.
- `avr_phase3_tune.m` sweeps detector and switcher parameters and saves the recommended configuration.
- `avr_validation_matrix.m` prints the consolidated attack-versus-metric summary.
- `phase3_full_run.m` writes the broader batch log, CSV summary, MAT summary, and scenario plots.
- `PHASE3_EXECUTION_GUIDE.md` documents run order, expected results, and debugging knobs.

Expected healthy behavior:

- Clean baseline signals should not trigger detection.
- Detections should occur after the configured attack start time.
- Switching should move to PID without chatter or control spikes.
- Validation should show bounded ITAE and finite detection delay.

---

## 9. Success Criteria

**Phase 3 is complete when**:

1. ✅ All three attack types injected and visible in measurement
2. ✅ Detector identifies all three attacks within 2 seconds
3. ✅ Switcher transitions smoothly between 2DoF and PID without control spikes
4. ✅ Poles remain in LHP throughout normal → attack → recovery
5. ✅ Validation matrix shows:
   - PID remains stable under all attacks (baseline)
   - 2DoF + Detector outperforms 2DoF alone (lower ITAE when detector works)
   - Detection latency $t_d \leq 2$ s for all attack types
6. ✅ Stability proof sketch included in documentation
7. ✅ All code follows correctness-first principles (no hardcoded magic numbers, all parameters named and documented)

---

## 10. Known Risks & Mitigation

| Risk | Impact | Mitigation |
|------|--------|-----------|
| Kalman filter mismatch | Residuals may not follow expected statistics | Use actual AVR plant for observer design; tune Q, R offline |
| Detection threshold too tight | False positives during normal transient | Baseline window must exclude rise-time transient (use t ∈ [4, 5]s) |
| Detection threshold too loose | Misses small attacks | Cross-validate threshold vs. attack magnitude; test with 0.05 pu bias |
| Switching delays | Control oscillation during transition | Implement bumpless transfer; log integral state and reuse |
| PID less optimal | ITAE post-detection higher than 2DoF | Acceptable trade-off for guaranteed stability |
| Multiple attack types in real data | Detector designed for single attack | Future work: multi-hypothesis tracking (Phase 4+) |

---

## 11. References & Validation Sources

- IEEE Type-1 AVR: *Power System Dynamics and Stability* (Sauer & Pai, 2006)
- Kalman Filter Residuals: *Optimal Filtering* (Anderson & Moore, 1979)
- 2DoF FOPID Stability: Xue et al., *Fractional-Order Control Systems*, 2016
- Switching Control: *Switched Linear Systems* (Liberzon, 2003)

---

**Next Step**: Proceed to implementation of modular files in order:  
1. `avr_attack_injector.m`  
2. `avr_detector.m`  
3. `avr_switcher.m`  
4. `avr_phase3_test.m`  
5. `avr_validation_matrix.m`  

All code will follow this architecture exactly. No shortcuts. ✓
