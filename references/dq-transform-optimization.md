# dq / Clarke / Park Transform Optimization (Reference)

## 1. Data Path Conventions

### Clarke Transform (abc → αβ)
- Converts three-phase (a,b,c) to two-phase stationary reference frame (α,β)
- Physical meaning: α represents real axis, β represents imaginary axis
- Reduces three coupled quantities to two independent quantities

### Park Transform (αβ → dq)
- Rotates stationary frame to synchronous rotating frame aligned with grid/rotor
- Physical meaning: d-axis tracks active power, q-axis tracks reactive power
- Converts AC quantities to DC → enables PI control (instead of PR)

### Inverse Park Transform (dq → αβ)
- Converts controller output back to stationary frame for PWM modulation

### Complete Implementation

```c
/**
 * @brief  Clarke transform: three-phase abc to stationary αβ frame.
 *         Assumes balanced three-phase system (ia + ib + ic = 0).
 *
 *         α = ia
 *         β = (ia + 2×ib) / √3
 *
 *         Using ia + ib + ic = 0 → ic = -(ia + ib) is implicit.
 *
 * @note   This is the power-invariant form (amplitude = √(2/3) × peak).
 *         Some implementations use 2/3 scaling — be consistent.
 */
typedef struct {
    float32_t alpha;
    float32_t beta;
} clarke_out_t;

static inline clarke_out_t clarke_transform(float32_t ia, float32_t ib) {
    clarke_out_t result;
    result.alpha = ia;
    result.beta  = (ia + 2.0f * ib) * 0.57735f;  /* 1/√3 = 0.57735 */
    return result;
}

/**
 * @brief  Full three-phase Clarke (when ia + ib + ic ≠ 0 exactly,
 *         e.g., sensor offsets or unbalanced load).
 */
static inline clarke_out_t clarke_transform_3ph(float32_t ia,
                                                 float32_t ib,
                                                 float32_t ic) {
    clarke_out_t result;
    result.alpha = (2.0f * ia - ib - ic) * 0.33333f;  /* 1/3 */
    result.beta  = (ib - ic) * 0.57735f;               /* 1/√3 */
    return result;
}

/**
 * @brief  Park transform: stationary αβ to synchronous dq frame.
 *         Requires angle θ from PLL (grid) or encoder (motor).
 *
 *         d =  α×cos(θ) + β×sin(θ)
 *         q = −α×sin(θ) + β×cos(θ)
 *
 * @note   cos_theta and sin_theta should be precomputed ONCE per ISR
 *         and reused for both current and voltage transforms.
 */
typedef struct {
    float32_t d;
    float32_t q;
} park_out_t;

static inline park_out_t park_transform(float32_t alpha, float32_t beta,
                                         float32_t cos_theta,
                                         float32_t sin_theta) {
    park_out_t result;
    result.d =  alpha * cos_theta + beta * sin_theta;
    result.q = -alpha * sin_theta + beta * cos_theta;
    return result;
}

/**
 * @brief  Inverse Park transform: synchronous dq to stationary αβ frame.
 *         Used to convert controller output to modulation reference.
 *
 *         α = d×cos(θ) − q×sin(θ)
 *         β = d×sin(θ) + q×cos(θ)
 */
static inline clarke_out_t inverse_park_transform(float32_t d, float32_t q,
                                                   float32_t cos_theta,
                                                   float32_t sin_theta) {
    clarke_out_t result;
    result.alpha = d * cos_theta - q * sin_theta;
    result.beta  = d * sin_theta + q * cos_theta;
    return result;
}
```

### Combined Transform Pipeline

```c
/**
 * @brief  Complete measurement pipeline: abc → αβ → dq in one call.
 *         Precompute sin/cos once for all transforms.
 *
 * @param  ia, ib       Phase currents [A]
 * @param  theta        PLL angle [rad]
 * @param  cos_theta    Precomputed cos(θ) [reuse for voltage transform]
 * @param  sin_theta    Precomputed sin(θ)
 * @return dq-frame current
 */
park_out_t abc_to_dq(float32_t ia, float32_t ib,
                      float32_t cos_theta, float32_t sin_theta) {
    clarke_out_t ab = clarke_transform(ia, ib);
    return park_transform(ab.alpha, ab.beta, cos_theta, sin_theta);
}

/**
 * @brief  Complete output pipeline: dq → αβ (→ SVM/SPWM).
 */
clarke_out_t dq_to_alphabeta(float32_t vd, float32_t vq,
                              float32_t cos_theta,
                              float32_t sin_theta) {
    return inverse_park_transform(vd, vq, cos_theta, sin_theta);
}
```

## 2. Angle Source and Synchronization

- Use PLL angle `theta` from grid tracking (grid-tied) or encoder (motor)
- Keep in same units (radians) and prevent drift: wrap to [0, 2π) each cycle
- If fast ISR and slow PLL run at different rates, interpolate angle between updates

### Angle Interpolation

```c
/**
 * @brief  Linear interpolation of PLL angle for fast ISR.
 *         PLL updates at slow rate (e.g., 10 kHz), ISR runs at 40 kHz.
 *         Interpolate to reduce angle quantization in fast loop.
 */
typedef struct {
    float32_t theta_pll;      /* Latest PLL angle [rad]              */
    float32_t omega_pll;      /* PLL estimated angular velocity [rad/s] */
    float32_t theta_interp;   /* Interpolated angle for ISR          */
    float32_t ts_fast;        /* Fast ISR period [s]                 */
} angle_interp_t;

static inline float32_t angle_interpolate(angle_interp_t *ai) {
    ai->theta_interp += ai->omega_pll * ai->ts_fast;

    /* Wrap to [0, 2π) */
    if (ai->theta_interp > 6.2832f) {
        ai->theta_interp -= 6.2832f;
    } else if (ai->theta_interp < 0.0f) {
        ai->theta_interp += 6.2832f;
    }

    return ai->theta_interp;
}

/* Call when PLL produces new update (slow loop) */
void angle_interp_sync(angle_interp_t *ai, float32_t theta_new,
                       float32_t omega_new) {
    ai->theta_pll    = theta_new;
    ai->theta_interp = theta_new;  /* Reset interpolation to PLL value */
    ai->omega_pll    = omega_new;
}
```

## 3. Normalization and Sequence

### Power-Invariant vs Amplitude-Invariant
- **Power-invariant** (2/3 scaling): preserves power across domains; `P = 1.5 × (Vd×Id + Vq×Iq)`
- **Amplitude-invariant** (√(2/3) scaling): peak values match between domains
- **Rule**: choose one convention and use consistently across entire project

### Positive and Negative Sequence
- Positive sequence: phasors rotate counter-clockwise in abc order
- Negative sequence: reversed sign for q-axis
- In unbalanced grid: both sequences present, use DSOGI-PLL or dual-frame for separation

## 4. Optimization Techniques

### Shared Trigonometric Computation

```c
/**
 * @brief  ISR execution pattern: compute sin/cos ONCE, reuse everywhere.
 *         STM32G4 CORDIC computes sin AND cos in a single operation.
 */
void control_isr(void) {
    /* [1] Get angle from PLL (or interpolation) */
    float32_t theta = angle_interpolate(&angle_state);

    /* [2] Compute sin/cos ONCE using CORDIC */
    float32_t sin_theta, cos_theta;
    cordic_sin_cos(theta, &sin_theta, &cos_theta);

    /* [3] Current measurement → dq */
    park_out_t i_dq = abc_to_dq(ia_meas, ib_meas, cos_theta, sin_theta);

    /* [4] Voltage measurement → dq (reuse same sin/cos) */
    park_out_t v_dq = abc_to_dq(va_meas, vb_meas, cos_theta, sin_theta);

    /* [5] Control loops in dq frame */
    float32_t vd_out = pi_update(&id_pi, id_ref, i_dq.d);
    float32_t vq_out = pi_update(&iq_pi, iq_ref, i_dq.q);

    /* [6] Inverse Park → αβ (reuse same sin/cos) */
    clarke_out_t v_ab = dq_to_alphabeta(vd_out, vq_out,
                                         cos_theta, sin_theta);

    /* [7] SVM → PWM update */
    svm_update(v_ab.alpha, v_ab.beta);
}
```

### Reducing Multiplications
- Clarke transform: only 2 multiplies (using √3 precomputed)
- Park transform: 4 multiplies (2 sin, 2 cos terms)
- Inverse Park: 4 multiplies (same as Park)
- Total arithmetic cost is modest, but final cycle count depends on inlining,
  memory access, compiler options, and whether trig comes from CORDIC or software.

### CORDIC vs Software Trigonometry

| Method | Cycle Count | Accuracy | Notes |
|--------|------------|----------|-------|
| `arm_sin_f32()` | ~15-20 | Full float | Software, CMSIS DSP |
| Sine LUT (1024 entries) | ~5-8 | 10-bit | Fixed memory cost |
| **STM32G4 CORDIC** | **Low, deterministic latency** | **Config-dependent** | **Recommended when available** |
| `sinf()` (libm) | ~30-50 | Full float | Avoid in ISR |

## 5. STM32G4 Guidance

- Use CORDIC for sin/cos when it is available and integration cost is justified
- Keep the full transform pipeline within the measured ISR budget; do not assume a fixed cycle count without profiling
- For high-speed (40 kHz): place transform functions in RAMFUNC section
- Use `static inline` for transform functions to avoid function call overhead
- If CPU load is critical: consider running transforms at half ISR rate (40 kHz → 20 kHz)
- Evaluate total pipeline timing with DWT cycle counter during development
