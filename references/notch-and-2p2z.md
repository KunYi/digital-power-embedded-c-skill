# Notch and IIR 2P2Z (Reference)

## 1. NOTCH Filter

### Physical Meaning
- Band-stop filter that attenuates a specific frequency (e.g., switching harmonics, grid harmonics, mechanical resonances)
- In digital power: reject 2× line frequency ripple on DC bus, attenuate specific harmonic components, suppress LC filter resonance

### Design Parameters
- **Center frequency ω₀**: frequency to reject
- **Bandwidth (BW)**: width of the notch; narrower = more selective but more sensitive to frequency drift
- **Notch depth**: attenuation at center frequency; typically -40 dB or better
- **Quality factor Q**: Q = ω₀ / BW; higher Q = narrower notch

### Complete Implementation

```c
/**
 * @brief  Notch filter using biquad structure.
 *         Rejects a specific frequency while passing all others.
 *
 *         Continuous form: H(s) = (s² + ω₀²) / (s² + (ω₀/Q)s + ω₀²)
 *         Discretized via bilinear (Tustin) transform.
 */
typedef struct {
    /* Coefficients */
    float32_t b0, b1, b2;
    float32_t a1, a2;

    /* State (Direct Form II Transposed) */
    float32_t s1, s2;
} notch_filter_t;

/**
 * @brief  Initialize notch filter.
 * @param  nf      Notch filter instance
 * @param  freq_hz Center frequency to reject [Hz]
 * @param  q       Quality factor (higher = narrower, typical: 5–30)
 * @param  fs_hz   Sampling frequency [Hz]
 */
void notch_init(notch_filter_t *nf, float32_t freq_hz,
                float32_t q, float32_t fs_hz) {
    float32_t w0 = 6.2832f * freq_hz / fs_hz;  /* Normalized frequency */
    float32_t cos_w0 = arm_cos_f32(w0);
    float32_t alpha = arm_sin_f32(w0) / (2.0f * q);

    float32_t a0_inv = 1.0f / (1.0f + alpha);

    nf->b0 =  1.0f * a0_inv;
    nf->b1 = -2.0f * cos_w0 * a0_inv;
    nf->b2 =  1.0f * a0_inv;
    nf->a1 = -2.0f * cos_w0 * a0_inv;
    nf->a2 =  (1.0f - alpha) * a0_inv;

    nf->s1 = 0.0f;
    nf->s2 = 0.0f;
}

/**
 * @brief  Notch filter update — Direct Form II Transposed.
 */
static inline float32_t notch_update(notch_filter_t *nf, float32_t input) {
    float32_t output = nf->b0 * input + nf->s1;
    nf->s1 = nf->b1 * input - nf->a1 * output + nf->s2;
    nf->s2 = nf->b2 * input - nf->a2 * output;
    return output;
}

/**
 * @brief  Reset notch filter states.
 */
static inline void notch_reset(notch_filter_t *nf) {
    nf->s1 = 0.0f;
    nf->s2 = 0.0f;
}
```

### Common Notch Applications in Digital Power

| Application | Center Frequency | Q | Notes |
|-------------|-----------------|---|-------|
| DC bus 2f ripple (50Hz) | 100 Hz | 10–20 | Reject 100Hz ripple from single-phase PFC |
| DC bus 2f ripple (60Hz) | 120 Hz | 10–20 | Same for 60Hz systems |
| 3rd harmonic rejection | 150/180 Hz | 15–25 | Grid harmonic rejection |
| Switching ripple | fsw (e.g., 40kHz) | 20–30 | Only if in ADC bandwidth |
| LC resonance damping | fr = 1/(2π√LC) | 5–10 | Active damping of output filter |

## 2. IIR 2P2Z Compensator

### Physical Meaning
- General-purpose two-pole, two-zero compensator for shaping control loop frequency response
- Equivalent to PID controller in different parameterization
- Used as type-II or type-III compensator in voltage/current loops

### Design from Continuous Domain

```c
/**
 * @brief  Type-II compensator (one pole at origin, one real pole, one real zero).
 *         Used for voltage loop compensation in buck/boost converters.
 *
 *         Continuous: H(s) = G0 × (1 + s/ωz) / (s × (1 + s/ωp))
 *
 *         Design steps:
 *         1. Choose crossover frequency fc (typically fsw/10 for voltage loop)
 *         2. Place zero at plant pole: ωz = 2π×fc / 3
 *         3. Place pole for high-frequency roll-off: ωp = 2π×fc × 3
 *         4. Set gain G0 for 0 dB crossing at fc
 *
 * @param  comp          2P2Z compensator (biquad)
 * @param  fc_hz         Desired crossover frequency [Hz]
 * @param  plant_gain_fc Plant gain at crossover [dB] (to set G0)
 * @param  fs_hz         Sampling frequency [Hz]
 */
void type2_comp_design(biquad_t *comp, float32_t fc_hz,
                        float32_t plant_gain_at_fc, float32_t fs_hz) {
    float32_t ts = 1.0f / fs_hz;

    /* Zero and pole placement */
    float32_t fz = fc_hz / 3.0f;    /* Zero at fc/3 */
    float32_t fp = fc_hz * 3.0f;     /* Pole at fc×3 */

    /* Bilinear transform: s = (2/Ts) × (z-1)/(z+1) */
    float32_t wz = 6.2832f * fz;
    float32_t wp = 6.2832f * fp;

    /* Pre-warped frequencies */
    float32_t wz_d = (2.0f / ts) * tanf(wz * ts * 0.5f);
    float32_t wp_d = (2.0f / ts) * tanf(wp * ts * 0.5f);

    /* Gain to achieve 0dB at crossover */
    float32_t g0 = 1.0f / plant_gain_at_fc;

    /* Discrete coefficients via Tustin transform */
    float32_t c1 = 2.0f / ts;
    float32_t num_a = c1 + wz_d;
    float32_t num_b = wz_d - c1;
    float32_t den_a = c1 + wp_d;
    float32_t den_b = wp_d - c1;

    comp->b0 = g0 * num_a / den_a;
    comp->b1 = g0 * num_b / den_a;
    comp->b2 = 0.0f;
    comp->a1 = den_b / den_a;
    comp->a2 = 0.0f;

    comp->s1 = 0.0f;
    comp->s2 = 0.0f;
}
```

### 2P2Z Update (Same as Biquad)

```c
/**
 * @brief  2P2Z compensator update — identical to biquad_update().
 *         Direct Form II Transposed:
 *           y₀ = b₀×x₀ + s₁
 *           s₁ = b₁×x₀ − a₁×y₀ + s₂
 *           s₂ = b₂×x₀ − a₂×y₀
 *
 * @note   Computation: 5 multiplies, 4 adds = ~8 cycles on FPU
 *         States s1, s2 MUST be cleared on fault to prevent wind-up
 */
static inline float32_t comp_2p2z_update(biquad_t *comp,
                                          float32_t error) {
    float32_t output = comp->b0 * error + comp->s1;
    comp->s1 = comp->b1 * error - comp->a1 * output + comp->s2;
    comp->s2 = comp->b2 * error - comp->a2 * output;
    return output;
}
```

## 3. Dual-Biquad Chain

### Higher-Order Compensation

```c
/**
 * @brief  Type-III compensator using cascade of two biquad sections.
 *         Provides additional phase boost compared to Type-II.
 *
 *         Continuous: H(s) = G0 × (1+s/ωz1)(1+s/ωz2) /
 *                                  (s × (1+s/ωp1)(1+s/ωp2))
 *
 * @note   Cascade order matters for numerical accuracy:
 *         place section with smallest gain first.
 */
typedef struct {
    biquad_t section1;       /* First biquad stage              */
    biquad_t section2;       /* Second biquad stage             */
    float32_t out_max;       /* Output saturation limit         */
    float32_t out_min;
} type3_comp_t;

float32_t type3_comp_update(type3_comp_t *comp, float32_t error) {
    float32_t stage1_out = comp_2p2z_update(&comp->section1, error);
    float32_t output = comp_2p2z_update(&comp->section2, stage1_out);

    /* Output saturation */
    if (output > comp->out_max) { output = comp->out_max; }
    if (output < comp->out_min) { output = comp->out_min; }

    return output;
}

void type3_comp_reset(type3_comp_t *comp) {
    biquad_reset(&comp->section1);
    biquad_reset(&comp->section2);
}
```

## 4. Discrete Coefficient Establishment

### Design Methodology
1. **Start with continuous H(s)**: Design in s-domain using Bode plot / loop-shaping
2. **Apply bilinear (Tustin) transform**: s = (2/Ts) × (z-1)/(z+1)
3. **Extract a, b coefficients**: Map to biquad structure
4. **Validate**: Check poles inside unit circle, verify frequency response matches design

### Coefficient Verification

```c
/**
 * @brief  Verify biquad stability (poles inside unit circle).
 * @return true if filter is stable
 *
 * @note   Instability indicators:
 *         - |a2| ≥ 1.0: poles on or outside unit circle
 *         - Output grows without bound
 *         - Audible oscillation or relay-like chattering
 */
static inline bool biquad_is_stable(const biquad_t *bq) {
    /* Sufficient condition: |a2| < 1 and |a1| < 1 + a2 */
    float32_t abs_a2 = fabsf(bq->a2);
    float32_t abs_a1 = fabsf(bq->a1);
    return (abs_a2 < 1.0f) && (abs_a1 < (1.0f + bq->a2));
}
```

## 5. STM32G4 Usage

- Prefer `arm_biquad_cascade_df2T_f32()` for multi-stage filtering in production
- For Q31 fixed-point: `arm_biquad_cascade_df2T_q31()` with proper block size
- Coefficient update (if adaptive): compute in float during slow task, convert to Q31 if needed
- Synchronize coefficient changes with control loops to avoid transient glitches
- Clear filter states on any coefficient change to prevent output spikes
- Typical budget: notch filter ≈ 10 cycles, dual-biquad chain ≈ 20 cycles on Cortex-M4 FPU
