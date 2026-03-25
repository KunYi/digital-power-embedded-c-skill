# LPF and RMS Variants (Reference)

## 1. First-Order LPF

### Physical Meaning
- Low-pass filter attenuates high-frequency noise while preserving DC/low-frequency components
- In digital power: smooth ADC readings, filter PWM ripple, extract DC components for control
- Trade-off: faster filtering (higher cutoff) = less noise rejection; slower = more phase delay

### Complete Implementation

```c
/**
 * @brief  First-order IIR low-pass filter.
 *         Discrete form: y(k) = α × y(k-1) + (1-α) × x(k)
 *         where α = exp(-2π × fc / fs)
 *
 *         Physical meaning: α closer to 1.0 = heavier filtering (lower cutoff)
 */
typedef struct {
    float32_t alpha;         /* Filter coefficient (pole location)   */
    float32_t one_minus_a;   /* Precomputed: 1 - alpha               */
    float32_t output;        /* Current filtered output               */
} lpf1_t;

/**
 * @brief  Initialize first-order LPF with cutoff frequency.
 * @param  lpf       Filter instance
 * @param  fc_hz     Cutoff frequency (-3dB point) [Hz]
 * @param  fs_hz     Sampling frequency [Hz]
 * @param  init_val  Initial output value
 */
void lpf1_init(lpf1_t *lpf, float32_t fc_hz, float32_t fs_hz,
               float32_t init_val) {
    /* α = exp(-2π × fc / fs) */
    /* Approximation for small fc/fs: α ≈ 1 - 2π×fc/fs (avoids expf) */
    float32_t wc_ts = 6.2832f * fc_hz / fs_hz;
    if (wc_ts < 0.5f) {
        /* Use approximation for efficiency (error < 1% when wc_ts < 0.3) */
        lpf->alpha = 1.0f - wc_ts;
    } else {
        /* Full computation for large cutoff ratios */
        lpf->alpha = expf(-wc_ts);
    }
    lpf->one_minus_a = 1.0f - lpf->alpha;
    lpf->output = init_val;
}

/**
 * @brief  First-order LPF update.
 *         2 multiplies, 1 add. Minimal cycle count.
 */
static inline float32_t lpf1_update(lpf1_t *lpf, float32_t input) {
    lpf->output = lpf->alpha * lpf->output + lpf->one_minus_a * input;
    return lpf->output;
}

/**
 * @brief  Reset filter output (call on fault or startup).
 */
static inline void lpf1_reset(lpf1_t *lpf, float32_t value) {
    lpf->output = value;
}
```

### Common Cutoff Frequency Settings

| Application | Cutoff Frequency | Rationale |
|-------------|-----------------|-----------|
| Bus voltage filtering | 100–500 Hz | Remove switching ripple, keep regulation dynamics |
| Current RMS | 10–50 Hz | Extract fundamental RMS over 1-2 AC cycles |
| Temperature sensing | 1–5 Hz | NTC noise rejection, slow thermal dynamics |
| PLL loop filter | 5–30 Hz | Reject harmonics while tracking grid |
| Power calculation | 10–100 Hz | Smooth instantaneous power for display/protection |

## 2. Multi-Stage LPF / RMS

### Cascaded LPF for Steeper Roll-off

```c
/**
 * @brief  Third-order LPF using cascade of three first-order filters.
 *         Provides -60 dB/decade roll-off vs -20 dB/decade for single.
 *
 * @note   Each stage adds phase delay. Total group delay ≈ 3 × (1 / (2π×fc)).
 *         For 10 Hz cutoff: ~48 ms total delay → acceptable for monitoring,
 *         NOT for fast control loops.
 */
#define LPF_CASCADE_STAGES  3U

typedef struct {
    lpf1_t stages[LPF_CASCADE_STAGES];
} lpf_cascade_t;

void lpf_cascade_init(lpf_cascade_t *lc, float32_t fc_hz,
                       float32_t fs_hz, float32_t init_val) {
    for (uint8_t i = 0U; i < LPF_CASCADE_STAGES; i++) {
        lpf1_init(&lc->stages[i], fc_hz, fs_hz, init_val);
    }
}

float32_t lpf_cascade_update(lpf_cascade_t *lc, float32_t input) {
    float32_t out = input;
    for (uint8_t i = 0U; i < LPF_CASCADE_STAGES; i++) {
        out = lpf1_update(&lc->stages[i], out);
    }
    return out;
}
```

### RMS Calculation

```c
/**
 * @brief  True RMS measurement using square-accumulate-sqrt method.
 *         Square and accumulate in fast ISR; sqrt in slow task.
 *
 *         RMS = √(mean(x²)) — measures signal magnitude regardless
 *         of waveform shape (works for sinusoidal, distorted, DC+AC).
 */
typedef struct {
    float32_t sum_sq;        /* Running sum of x²                    */
    uint32_t  sample_count;  /* Number of samples accumulated        */
    uint32_t  window_size;   /* Samples per RMS window               */
    float32_t rms_value;     /* Last computed RMS result              */
} rms_calc_t;

/**
 * @brief  Initialize RMS calculator.
 * @param  rms         RMS calculator instance
 * @param  window_ms   Averaging window [ms] (should be integer
 *                     multiple of AC period for accurate result)
 * @param  fs_hz       Sampling frequency [Hz]
 */
void rms_init(rms_calc_t *rms, float32_t window_ms, float32_t fs_hz) {
    rms->window_size = (uint32_t)(window_ms * 0.001f * fs_hz);
    if (rms->window_size == 0U) { rms->window_size = 1U; }
    rms->sum_sq = 0.0f;
    rms->sample_count = 0U;
    rms->rms_value = 0.0f;
}

/**
 * @brief  Accumulate sample in ISR (fast: 1 multiply + 1 add).
 */
static inline void rms_accumulate(rms_calc_t *rms, float32_t sample) {
    rms->sum_sq += sample * sample;
    rms->sample_count++;
}

/**
 * @brief  Compute RMS — call from slow task (1ms or per-window).
 *         Contains sqrtf(), so keep it out of the fast ISR unless target
 *         profiling shows the cost is acceptable.
 * @return true if new RMS value is available
 */
bool rms_compute(rms_calc_t *rms) {
    if (rms->sample_count >= rms->window_size) {
        float32_t mean_sq = rms->sum_sq / (float32_t)rms->sample_count;
        rms->rms_value = sqrtf(mean_sq);
        rms->sum_sq = 0.0f;
        rms->sample_count = 0U;
        return true;
    }
    return false;
}
```

### Sliding Window RMS (Alternative)

```c
/**
 * @brief  Sliding window RMS using circular buffer.
 *         More responsive than block-based but uses more memory.
 *
 * @note   Memory usage: window_size × 4 bytes for float32_t buffer.
 *         For 20ms window at 40kHz = 800 samples = 3200 bytes.
 */
#define RMS_WINDOW_SIZE  800U  /* 20ms at 40kHz */

typedef struct {
    float32_t buffer[RMS_WINDOW_SIZE];
    float32_t sum_sq;
    uint16_t  index;
    bool      buffer_full;
} rms_sliding_t;

float32_t rms_sliding_update(rms_sliding_t *rms, float32_t sample) {
    float32_t sq = sample * sample;

    /* Subtract oldest, add newest */
    if (rms->buffer_full) {
        rms->sum_sq -= rms->buffer[rms->index];
    }
    rms->buffer[rms->index] = sq;
    rms->sum_sq += sq;

    rms->index++;
    if (rms->index >= RMS_WINDOW_SIZE) {
        rms->index = 0U;
        rms->buffer_full = true;
    }

    uint16_t count = rms->buffer_full ? RMS_WINDOW_SIZE : rms->index;
    return sqrtf(rms->sum_sq / (float32_t)count);
}
```

## 3. Coefficient Pre-calculation

```c
/**
 * @brief  Pre-calculate LPF coefficients for common cutoff frequencies.
 *         Store in const table to avoid runtime expf/division.
 *
 * @note   Use when filter cutoff is selected from fixed set of values
 *         (e.g., bus voltage filter: 100/200/500 Hz options).
 */
typedef struct {
    float32_t fc_hz;         /* Cutoff frequency label               */
    float32_t alpha;         /* Pre-calculated coefficient           */
    float32_t one_minus_a;   /* Pre-calculated 1 - alpha             */
} lpf_coeff_table_t;

/* Example: coefficients for fs = 40kHz */
static const lpf_coeff_table_t lpf_table_40khz[] = {
    { .fc_hz =  10.0f, .alpha = 0.99843f, .one_minus_a = 0.00157f },
    { .fc_hz =  50.0f, .alpha = 0.99215f, .one_minus_a = 0.00785f },
    { .fc_hz = 100.0f, .alpha = 0.98432f, .one_minus_a = 0.01568f },
    { .fc_hz = 500.0f, .alpha = 0.92419f, .one_minus_a = 0.07581f },
    { .fc_hz = 1000.0f,.alpha = 0.85464f, .one_minus_a = 0.14536f },
};
```

## 4. Extreme LPF (Minimal Cycle Count)

```c
/**
 * @brief  Extreme LPF: single MAC operation for fixed coefficient.
 *         Use when α is known at compile time and cycle budget is critical.
 *
 *         Hardcoded example: 100 Hz cutoff at 40 kHz sample rate.
 */
#define LPF_ALPHA_100HZ    (0.98432f)
#define LPF_1MA_100HZ      (0.01568f)

static float32_t lpf_extreme_state;

__attribute__((section(".ramfunc")))
static inline float32_t lpf_extreme_update(float32_t input) {
    lpf_extreme_state = LPF_ALPHA_100HZ * lpf_extreme_state
                      + LPF_1MA_100HZ * input;
    return lpf_extreme_state;
}
```

## 5. Fault Filtering and System Safety

- **Input Validation**: Disable filter output if ADC input is out-of-range or sampling fault detected
- **Fault Decision**: Use slow 2nd-order LPF (τ = 10ms~100ms) for reliable fault confirmation — prevents nuisance trips from transient noise
- **Safety Integration**: Filter outputs should feed protection logic with appropriate time constants — faster filter for OCP, slower for OTP
- **State Reset**: Clear all filter states on fault recovery to avoid stale data affecting restart

## 6. STM32G4 Recommendations

- Use `arm_fir_f32()` from CMSIS DSP for FIR filters, `arm_rms_f32()` for block RMS
- Sync DMA+ADC trigger with TIM1/TIM8 PWM for simultaneous sampling
- For RMS: accumulate in ISR (fast), compute sqrt in 1ms task (slow) — never sqrtf in fast ISR
- Consider hardware oversampling (ADC CFGR2) as alternative to software LPF for noise reduction
