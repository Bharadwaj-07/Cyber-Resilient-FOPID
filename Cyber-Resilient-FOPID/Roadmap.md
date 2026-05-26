# Roadmap

## Visual Roadmap (Build Order)

```mermaid
graph TD
    P1[Phase 1: AVR Plant Model]
    P2A[Phase 2: FOPID Operator (Oustaloup)]
    P2B[Phase 2: 2DoF Wrapper]
    P2C[Phase 2: PSO Tuner]
    P3A[Phase 3: Attack Injector]
    P3B[Phase 3: Detector]
    P4[Phase 4: Resilient Switching Logic]
    P5[Phase 5: Validation Matrix]

    P1 --> P2A
    P1 --> P2B
    P2A --> P2B
    P2B --> P2C
    P2C --> P3A
    P2C --> P3B
    P3A --> P4
    P3B --> P4
    P4 --> P5
```

## Logic Behind the Ordering

### Phase 1 -- Plant Model First
- AVR plant model is non-negotiable as the first step.
- Use IEEE Type-1 or ST1A exciter model for literature comparability.
- Validate with a clean step response before any controller work.

### Phase 2 -- Core Controller (Parallel Workstreams)
- Build three parallel components: FOPID numerical implementation, 2DoF wrapper, and tuner.
- Use Oustaloup's recursive approximation for the fractional operators.
- Wire the tuner last since it depends on the closed-loop evaluation.
- Objective function should be ITAE for meaningful settling-time penalties.

### Phase 3 -- Attack Layer
- Build the injector first as a switchable bias on the measurement feedback path.
- Build the detector independently to benchmark detection latency in isolation.

### Phase 4 -- Resilient Control
- Switching logic should be hysteresis-based to avoid chatter.
- Stability proof for the switching system under bounded attack is the core theoretical contribution.

### Phase 5 -- Validation Matrix
- Test at minimum three attack types: step bias, ramp, sinusoidal.
- Compare four systems: classical PID, 1DoF FOPID, 2DoF FOPID (no resilience), full resilient system.
- The 2x2 comparison matrix makes the evaluation rigorous.
