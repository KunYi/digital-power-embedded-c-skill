# Filter and Resonant Variants (Reference)

## 1. PR Controller (Proportional-Resonant)

### Physical Meaning
- Provides infinite gain at specific frequency (grid frequency) for zero steady-state error in AC systems
- Alternative to PI in synchronous (dq) frame: PR works directly in stationary (αβ) frame
- Used in single-phase inverters where dq transform requires orthogonal signal generation

### Discrete Transfer Function
- **Continuous**: G_pr(s) = Kp + Kr × 2ωc×s / (s² + 2ωc×s + ω₀²)
- **Discrete (Tustin)**: G_pr(z) = Kp + Kr × z⁻¹×(1+z⁻¹) / (1-2cos(ω₀Ts)z⁻¹+z⁻²)
- Where ω₀ = grid frequency (2π×50 or 2π×60), ωc = resonant bandwidth

### Complete Implementation

```c
/**
 * @brief  PR (Proportional-Resonant) controller structure.
 *         Resonant term provides infinite gain at target frequency ω₀
 *         for zero steady-state error in AC current/voltage tracking.
 */
typedef struct {
    /* Configuration */
    float32_t kp;           /* Proportional gain                    */
    float32_t kr;           /* Resonant gain                        */
    float32_t omega_0;      /* Target frequency [rad/s]             */
    float32_t omega_c;      /* Resonant bandwidth [rad/s]           */
    float32_t ts;           /* Sample period [s]                    */
    float32_t out_max;      /* Output upper clamp                   */
    float32_t out_min;      /* Output lower clamp                   */

    /* Precomputed coefficients (computed at init) */
    float32_t a1, a2;       /* Denominator coefficients             */
    float32_t b0, b1, b2;   /* Numerator coefficients               */

    /* State variables */
    float32_t s1, s2;       /* Resonant integrator states           */
    float32_t output;       /* Last output for monitoring           */
} pr_controller_t;

/**
 * @brief  Initialize PR controller and precompute discrete coefficients.
 *         Uses Tustin (bilinear) transform for discretization.
 */
void pr_init(pr_controller_t *pr, float32_t kp, float32_t kr,
             float32_t freq_hz, float32_t bandwidth_hz,
             float32_t ts, float32_t out_min, float32_t out_max) {
    pr->kp      = kp;
    pr->kr      = kr;
    pr->omega_0 = 6.2832f * freq_hz;
    pr->omega_c = 6.2832f * bandwidth_hz;
    pr->ts      = ts;
    pr->out_max = out_max;
    pr->out_min = out_min;

    /* Precompute Tustin-discretized resonant coefficients */
    float32_t w0_ts = pr->omega_0 * ts;
    float32_t wc_ts = pr->omega_c * ts;
    float32_t cos_w0 = arm_cos_f32(w0_ts);
    float32_t denom = 1.0f + wc_ts + w0_ts * w0_ts * 0.25f;

    pr->a1 = (2.0f * (w0_ts * w0_ts * 0.25f - 1.0f)) / denom;
    pr->a2 = (1.0f - wc_ts + w0_ts * w0_ts * 0.25f) / denom;
    pr->b0 = (kr * wc_ts) / denom;
    pr->b1 = 0.0f;
    pr->b2 = -(kr * wc_ts) / denom;

    /* Clear states */
    pr->s1 = 0.0f;
    pr->s2 = 0.0f;
    pr->output = 0.0f;
}

/**
 * @brief  PR controller update — Direct Form II Transposed.
 *         P-term is computed separately; resonant term uses biquad.
 *
 * @param  pr     PR controller instance
 * @param  error  Tracking error (reference - feedback)
 * @return Saturated output
 */
float32_t pr_update(pr_controller_t *pr, float32_t error) {
    /* Resonant term: Direct Form II Transposed biquad */
    float32_t resonant_out = pr->b0 * error + pr->s1;
    pr->s1 = pr->b1 * error - pr->a1 * resonant_out + pr->s2;
    pr->s2 = pr->b2 * error - pr->a2 * resonant_out;

    /* Total output = P + Resonant */
    float32_t out = pr->kp * error + resonant_out;

    /* Saturation */
    if (out > pr->out_max) { out = pr->out_max; }
    if (out < pr->out_min) { out = pr->out_min; }

    pr->output = out;
    return out;
}

/**
 * @brief  Reset PR resonant states on fault or mode change.
 */
void pr_reset(pr_controller_t *pr) {
    pr->s1 = 0.0f;
    pr->s2 = 0.0f;
    pr->output = 0.0f;
}
```

### Execution Strategy
- Keep P-term in fast ISR, update resonant integrator in same ISR for tracking accuracy
- If cycle budget is tight, update resonant term at half the ISR rate (every other cycle)
- Anti-windup: clamp resonant output and saturate combined output

## 2. General Biquad (IIR 2P2Z) Filter

### Structure

```c
/**
 * @brief  General-purpose biquad (2-pole, 2-zero) IIR filter.
 *         Used for NOTCH, bandpass, low-pass, and compensator designs.
 *         Direct Form II Transposed for numerical stability.
 *
 *         Transfer function: H(z) = (b0 + b1*z⁻¹ + b2*z⁻²) /
 *                                    (1  + a1*z⁻¹ + a2*z⁻²)
 */
typedef struct {
    /* Coefficients — precomputed at init */
    float32_t b0, b1, b2;   /* Numerator (feedforward)             */
    float32_t a1, a2;        /* Denominator (feedback), a0 = 1     */

    /* State variables */
    float32_t s1, s2;        /* Transposed form delay states        */
} biquad_t;

/**
 * @brief  Biquad filter update — Direct Form II Transposed.
 *         Computation: 5 multiplies, 4 adds per sample.
 */
static inline float32_t biquad_update(biquad_t *bq, float32_t input) {
    float32_t output = bq->b0 * input + bq->s1;
    bq->s1 = bq->b1 * input - bq->a1 * output + bq->s2;
    bq->s2 = bq->b2 * input - bq->a2 * output;
    return output;
}

/**
 * @brief  Reset biquad filter states (call on fault or mode change).
 */
static inline void biquad_reset(biquad_t *bq) {
    bq->s1 = 0.0f;
    bq->s2 = 0.0f;
}
```

### Cascaded Biquad Chain

```c
/**
 * @brief  Cascade multiple biquad stages for higher-order filtering.
 *         Each stage is independent; total order = 2 × num_stages.
 */
#define MAX_BIQUAD_STAGES  4U

typedef struct {
    biquad_t stages[MAX_BIQUAD_STAGES];
    uint8_t  num_stages;
} biquad_cascade_t;

float32_t biquad_cascade_update(biquad_cascade_t *bc, float32_t input) {
    float32_t out = input;
    for (uint8_t i = 0U; i < bc->num_stages; i++) {
        out = biquad_update(&bc->stages[i], out);
    }
    return out;
}
```

## 3. Resonant Filter Optimization

### Trigonometric Optimization
- For fixed ω₀: precompute cos(ω₀Ts) at init, avoid runtime trig calls
- Use identity cos(2θ) = 2cos²θ - 1 to reduce operations for multi-harmonic
- On STM32G4: use CORDIC for trig if frequency is adaptive (e.g., PLL-tracked)

### Complex Vector Resonator

```c
/**
 * @brief  Complex-vector resonator for fixed-frequency tracking.
 *         Avoids explicit sin/cos calls; uses rotation matrix.
 *         Good for PR controllers at known grid frequency.
 */
typedef struct {
    float32_t cos_w0ts;     /* Precomputed cos(ω₀×Ts) */
    float32_t sin_w0ts;     /* Precomputed sin(ω₀×Ts) */
    float32_t xr, xi;       /* Real and imaginary state */
} complex_resonator_t;

void complex_resonator_init(complex_resonator_t *cr,
                            float32_t freq_hz, float32_t ts) {
    float32_t w0ts = 6.2832f * freq_hz * ts;
    cr->cos_w0ts = arm_cos_f32(w0ts);
    cr->sin_w0ts = arm_sin_f32(w0ts);
    cr->xr = 0.0f;
    cr->xi = 0.0f;
}

float32_t complex_resonator_update(complex_resonator_t *cr,
                                   float32_t input) {
    /* Rotation: z(k) = R × z(k-1) + input */
    float32_t xr_new = cr->cos_w0ts * cr->xr - cr->sin_w0ts * cr->xi + input;
    float32_t xi_new = cr->sin_w0ts * cr->xr + cr->cos_w0ts * cr->xi;
    cr->xr = xr_new;
    cr->xi = xi_new;
    return cr->xr;
}
```

## 4. Implementation Notes

- **State Management**: Keep filter state as struct for easy reset and predictable sampling order
- **Numerical Stability**: Direct Form II Transposed recommended for floating-point; monitor for quantization effects in fixed-point
- **Overflow Protection**: Add output saturation after each biquad stage if processing large signals
- **Coefficient Validation**: Check that poles are inside unit circle (|a2| < 1) to ensure stability

## 5. STM32G4 Points

- CMSIS DSP: use `arm_biquad_cascade_df2T_f32()` for optimized multi-stage filtering
- FPU flags: `-mfpu=fpv4-sp-d16 -mfloat-abi=hard` mandatory for float biquad performance
- CORDIC: use for adaptive-frequency resonator coefficient update (when PLL tracks grid)
- Avoid branch-heavy fault handling in fast ISR (≥16 kHz); defer safety checks to 1ms task
- Typical cycle budget: single biquad ≈ 8-10 cycles on Cortex-M4 FPU
