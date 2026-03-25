# PI Code Variants (Reference)

## Selection Guide

### Choose Standard PI When
- Project is still in functional debugging phase
- Need to prioritize troubleshooting control logic issues
- ISR budget is still available
- Team has multiple members and needs high readability

### Choose Fast-path PI When
- PI structure has been confirmed correct
- ISR cycle budget is getting tight
- Need to accelerate without significantly sacrificing readability

### Choose Extreme Hot-path PI When
- Bottleneck has been clearly identified in PI itself
- Coefficients and limits are stable
- Team can accept specialized implementation
- There is a clear validation plan

## 1. Standard PI — Full Implementation

### Data Structure

```c
/**
 * @brief  Standard PI controller structure.
 *         All fields have explicit physical meaning for debugging.
 */
typedef struct {
    /* Configuration — set once at init, update in slow task only */
    float32_t kp;           /* Proportional gain                    */
    float32_t ki;           /* Integral gain                        */
    float32_t ki_ts;        /* Precomputed: Ki × Ts (avoid ISR mul) */
    float32_t out_max;      /* Output upper clamp [engineering unit] */
    float32_t out_min;      /* Output lower clamp [engineering unit] */

    /* State — updated every ISR call */
    float32_t integral;     /* Integrator accumulator               */
    float32_t output;       /* Last output (for monitoring)         */
    float32_t error;        /* Last error  (for monitoring)         */

    /* Anti-windup */
    float32_t kb;           /* Back-calculation coefficient [0.1~1] */
    bool      saturated;    /* true if output is clamped            */
} pi_controller_t;
```

### Initialization

```c
/**
 * @brief  Initialize PI controller with safety defaults.
 * @param  pi       PI controller instance
 * @param  kp       Proportional gain
 * @param  ki       Integral gain
 * @param  ts       Sample period [s] (e.g., 1/40000 for 40kHz)
 * @param  out_min  Output lower limit
 * @param  out_max  Output upper limit
 * @param  kb       Anti-windup back-calculation gain [0.1..1.0]
 */
void pi_init(pi_controller_t *pi, float32_t kp, float32_t ki,
             float32_t ts, float32_t out_min, float32_t out_max,
             float32_t kb) {
    pi->kp       = kp;
    pi->ki       = ki;
    pi->ki_ts    = ki * ts;   /* Precompute for ISR */
    pi->out_max  = out_max;
    pi->out_min  = out_min;
    pi->kb       = kb;
    pi->integral = 0.0f;
    pi->output   = 0.0f;
    pi->error    = 0.0f;
    pi->saturated = false;
}
```

### Update Function

```c
/**
 * @brief  Standard PI update with anti-windup back-calculation.
 *         Execution order: error → P-term → I-accumulate →
 *         sum → saturate → back-calculate → store
 *
 * @param  pi        PI controller instance
 * @param  reference Reference input [engineering unit]
 * @param  feedback  Feedback measurement [engineering unit]
 * @return Saturated output [engineering unit]
 *
 * @note   Cycle count: ~15-20 cycles on STM32G4 FPU
 */
float32_t pi_update(pi_controller_t *pi, float32_t reference,
                    float32_t feedback) {
    /* [1] Error calculation */
    pi->error = reference - feedback;

    /* [2] Proportional term */
    float32_t p_term = pi->kp * pi->error;

    /* [3] Integrator accumulation */
    pi->integral += pi->ki_ts * pi->error;

    /* [4] Unsaturated output */
    float32_t u_unsat = p_term + pi->integral;

    /* [5] Output saturation */
    float32_t u_sat;
    if (u_unsat > pi->out_max) {
        u_sat = pi->out_max;
        pi->saturated = true;
    } else if (u_unsat < pi->out_min) {
        u_sat = pi->out_min;
        pi->saturated = true;
    } else {
        u_sat = u_unsat;
        pi->saturated = false;
    }

    /* [6] Anti-windup back-calculation */
    if (pi->saturated) {
        pi->integral += pi->kb * (u_sat - u_unsat);
    }

    /* [7] Store output for monitoring */
    pi->output = u_sat;
    return u_sat;
}
```

### Integrator Reset (Protection Event)

```c
/**
 * @brief  Reset PI integrator on fault or state transition.
 *         Call when entering FAULT state or switching control mode.
 *
 * @param  pi          PI controller instance
 * @param  init_value  Initial integrator value (0 or steady-state estimate)
 */
void pi_reset(pi_controller_t *pi, float32_t init_value) {
    pi->integral  = init_value;
    pi->output    = 0.0f;
    pi->error     = 0.0f;
    pi->saturated = false;
}
```

## 2. Fast-path PI

```c
/**
 * @brief  Fast-path PI: minimized branches, precomputed constants.
 *         Suitable for 20-40 kHz current loop ISR.
 *
 * @note   Anti-windup uses simple integrator clamping (no back-calc)
 *         to eliminate extra multiply. Monitoring deferred to slow loop.
 *         Cycle count: ~8-12 cycles on STM32G4 FPU.
 */
typedef struct {
    float32_t kp;
    float32_t ki_ts;        /* Precomputed: Ki × Ts               */
    float32_t out_max;
    float32_t out_min;
    float32_t integral;
    float32_t int_max;      /* Integrator clamp (prevent windup)   */
    float32_t int_min;
} pi_fast_t;

static inline float32_t pi_fast_update(pi_fast_t *pi,
                                        float32_t error) {
    /* P + I accumulation */
    pi->integral += pi->ki_ts * error;

    /* Integrator clamp (branchless-friendly on Cortex-M4) */
    if (pi->integral > pi->int_max) { pi->integral = pi->int_max; }
    if (pi->integral < pi->int_min) { pi->integral = pi->int_min; }

    /* Output with saturation */
    float32_t out = pi->kp * error + pi->integral;
    if (out > pi->out_max) { out = pi->out_max; }
    if (out < pi->out_min) { out = pi->out_min; }

    return out;
}
```

## 3. Extreme Hot-path PI

```c
/**
 * @brief  Extreme hot-path PI for cycle-critical ISRs.
 *         Minimal register traffic, no function call overhead.
 *         All state and monitoring deferred to slow loop.
 *
 * @note   Use ONLY when profiling confirms PI is the bottleneck.
 *         Place in RAMFUNC section for zero flash wait states.
 *         Cycle count: ~5-7 cycles on STM32G4 FPU.
 *
 *   IMPORTANT: This sacrifices debuggability. The integrator state
 *   is only readable via memory watch or slow-loop snapshot.
 */

/* Globals for minimal register access in ISR */
static float32_t pi_extreme_integral;
static float32_t pi_extreme_kp;
static float32_t pi_extreme_ki_ts;
static float32_t pi_extreme_out_max;
static float32_t pi_extreme_out_min;

__attribute__((section(".ramfunc")))
static inline float32_t pi_extreme_update(float32_t error) {
    pi_extreme_integral += pi_extreme_ki_ts * error;
    float32_t out = pi_extreme_kp * error + pi_extreme_integral;

    /* Branchless-style clamp using ternary (compiler optimizes) */
    out = (out > pi_extreme_out_max) ? pi_extreme_out_max :
          (out < pi_extreme_out_min) ? pi_extreme_out_min : out;

    return out;
}
```

## 4. Anti-windup Variants

### Simple Integrator Clamp
```c
/* Fastest: just limit the integrator directly */
if (integral > int_max) { integral = int_max; }
if (integral < int_min) { integral = int_min; }
```

### Back-Calculation (Recommended for Standard PI)
```c
/* More accurate: adjust integrator based on saturation amount */
/* Kb typically 0.1 (slow recovery) to 1.0 (fast recovery)    */
float32_t u_unsat = p_term + integral;
float32_t u_sat = clamp(u_unsat, out_min, out_max);
integral += kb * (u_sat - u_unsat);
```

### Conditional Integration
```c
/* Stop integrating when output is saturated in same direction as error */
if (!((u_sat >= out_max && error > 0.0f) ||
      (u_sat <= out_min && error < 0.0f))) {
    integral += ki_ts * error;
}
```

## 5. Input Anomaly Protection

```c
/**
 * @brief  Validate PI inputs before computation.
 *         Detect NaN, Inf, or out-of-range values.
 *         Set soft-fault flag if anomaly detected.
 */
static inline bool pi_input_valid(float32_t value, float32_t range_max) {
    /* Check for NaN: NaN != NaN is true */
    if (value != value) { return false; }

    /* Check for Inf */
    if ((value > 1.0e30f) || (value < -1.0e30f)) { return false; }

    /* Check range */
    if ((value > range_max) || (value < -range_max)) { return false; }

    return true;
}
```

## 6. CMSIS DSP / Fixed-point / CORDIC Support

### Fixed-point Q31 PI
```c
/**
 * @brief  Fixed-point PI using Q1.31 format.
 *         Use when FPU is not available or deterministic timing is required.
 *
 * @note   Overflow handling: use saturating arithmetic from CMSIS DSP.
 *         arm_add_q31(), arm_mult_q31() provide saturation.
 */
#include "arm_math.h"

typedef struct {
    q31_t kp_q31;
    q31_t ki_ts_q31;
    q31_t integral_q31;
    q31_t out_max_q31;
    q31_t out_min_q31;
} pi_q31_t;

q31_t pi_q31_update(pi_q31_t *pi, q31_t error) {
    q31_t p_term;
    arm_mult_q31(&pi->kp_q31, &error, &p_term, 1);

    q31_t i_inc;
    arm_mult_q31(&pi->ki_ts_q31, &error, &i_inc, 1);
    pi->integral_q31 = __QADD(pi->integral_q31, i_inc);  /* Saturating add */

    q31_t output = __QADD(p_term, pi->integral_q31);

    /* Clamp */
    if (output > pi->out_max_q31) { output = pi->out_max_q31; }
    if (output < pi->out_min_q31) { output = pi->out_min_q31; }

    return output;
}
```

## 7. STM32G4 Recommendations

- Use `float32_t` (FPU single-precision) for all PI computations
- Compiler flags: `-mcpu=cortex-m4 -mfpu=fpv4-sp-d16 -mfloat-abi=hard`
- ISR policy: PI computation should complete in < 1/3 of switching period
- Place extreme PI in `.ramfunc` section for zero wait states from flash
- Use DWT cycle counter to verify actual cycle budget during development
- Coordinate integrator reset across voltage/current loops on protection events
