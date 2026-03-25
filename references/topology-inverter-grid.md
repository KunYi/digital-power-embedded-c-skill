# Topology: Inverter — Grid-Tied and Off-Grid (Reference)

## Overview

Inverters convert DC to AC for grid-connected or standalone applications. Grid-tied inverters must synchronize with utility grid voltage/frequency and meet power quality standards (IEEE 1547, IEC 61727). Off-grid (standalone) inverters must generate stable voltage/frequency autonomously under varying loads.

## 1. Grid-Tied Inverter — Current Control Mode

### Control Architecture

```
  Vdc ──► │ H-Bridge / │     L_filter      Grid
           │ 3-Phase    │────┤├────────── Vgrid
           │ Inverter   │                   │
           └────────────┘                   │
                 ▲                          │
                 │ PWM                      │
           ┌─────┴──────┐                  │
           │ Inverse     │    ┌─────────┐  │
           │ Park        │◄───┤ Current │◄─┤ i_meas
           │ Transform   │    │ Loop    │  │
           └─────────────┘    │ (dq PI) │  │
                              └────┬────┘  │
                                   │       │
                              ┌────▼────┐  │
                              │ PLL     │◄─┘
                              │ (Grid   │
                              │  Sync)  │
                              └─────────┘
```

### dq-Frame Current Control

```c
/**
 * @brief  Grid-tied inverter current control in synchronous dq frame.
 *         d-axis: active power (tracks Id_ref from MPPT or DC bus loop)
 *         q-axis: reactive power (typically Iq_ref = 0 for unity PF)
 *
 * @note   Decoupling terms (ωL × Id/Iq cross-coupling) improve
 *         dynamic response and reduce current THD.
 */
typedef struct {
    pi_controller_t id_pi;        /* d-axis current loop           */
    pi_controller_t iq_pi;        /* q-axis current loop           */
    pi_controller_t vdc_pi;       /* DC bus voltage loop           */
    float32_t id, iq;             /* Measured dq currents [A]      */
    float32_t id_ref, iq_ref;     /* Current references [A]        */
    float32_t vd_ff, vq_ff;       /* Grid voltage feedforward [V]  */
    float32_t vd_out, vq_out;     /* Controller output voltages [V]*/
    float32_t omega_l;            /* ω × L_filter for decoupling   */
    float32_t theta;              /* PLL angle [rad]               */
} grid_inverter_t;

/* DC bus voltage loop → generates d-axis current reference */
void grid_inv_dc_loop(grid_inverter_t *inv, float32_t vdc,
                      float32_t vdc_ref) {
    inv->id_ref = pi_update(&inv->vdc_pi, vdc_ref, vdc);
}

/* dq current loop — fast ISR (10–40 kHz) */
void grid_inv_current_loop(grid_inverter_t *inv) {
    float32_t vd_pi = pi_update(&inv->id_pi, inv->id_ref, inv->id);
    float32_t vq_pi = pi_update(&inv->iq_pi, inv->iq_ref, inv->iq);

    /* Cross-coupling decoupling + grid voltage feedforward */
    inv->vd_out = vd_pi - inv->omega_l * inv->iq + inv->vd_ff;
    inv->vq_out = vq_pi + inv->omega_l * inv->id + inv->vq_ff;

    /* Apply inverse Park transform → αβ → PWM modulation */
}
```

### Grid Voltage Feedforward

```c
/**
 * @brief  Feedforward grid voltage to controller output.
 *         Reduces current tracking error and improves disturbance rejection.
 *
 * @note   Feedforward should use filtered grid voltage (LPF, ~1kHz BW)
 *         to avoid amplifying grid harmonics into the PWM output.
 */
void grid_inv_feedforward(grid_inverter_t *inv,
                          float32_t vgrid_alpha, float32_t vgrid_beta) {
    /* Park transform of grid voltage to get Vd_ff, Vq_ff */
    float32_t cos_theta, sin_theta;
    cordic_sin_cos(inv->theta, &sin_theta, &cos_theta);

    inv->vd_ff =  vgrid_alpha * cos_theta + vgrid_beta * sin_theta;
    inv->vq_ff = -vgrid_alpha * sin_theta + vgrid_beta * cos_theta;
}
```

## 2. Off-Grid Inverter — Voltage Control Mode

### Control Architecture

```c
/**
 * @brief  Off-grid inverter generates its own AC voltage reference.
 *         Outer loop: voltage PI regulates output voltage
 *         Inner loop: current PI provides fast current limiting
 *
 * @note   Voltage reference is internally generated sinusoid.
 *         Frequency is fixed (50/60Hz) from internal oscillator.
 *         Load-dependent frequency droop can be added for parallel operation.
 */
typedef struct {
    pi_controller_t vd_pi;       /* d-axis voltage loop           */
    pi_controller_t vq_pi;       /* q-axis voltage loop           */
    pi_controller_t id_pi;       /* d-axis inner current loop     */
    pi_controller_t iq_pi;       /* q-axis inner current loop     */
    float32_t vd_ref;            /* d-axis voltage ref (e.g., 325V peak) */
    float32_t vq_ref;            /* q-axis voltage ref (typically 0)     */
    float32_t vd, vq;            /* Measured output voltage in dq  */
    float32_t id, iq;            /* Measured inductor current in dq */
    float32_t theta;             /* Self-generated angle [rad]     */
    float32_t omega;             /* Angular frequency [rad/s]      */
} offgrid_inverter_t;

/* Angle generator — runs in slow loop or fast loop */
void offgrid_angle_update(offgrid_inverter_t *inv, float32_t dt) {
    inv->theta += inv->omega * dt;
    if (inv->theta > 6.2832f) {
        inv->theta -= 6.2832f;
    }
}

/* Dual-loop control in dq frame */
void offgrid_control_loop(offgrid_inverter_t *inv) {
    /* Outer voltage loop → current reference */
    float32_t id_ref = pi_update(&inv->vd_pi, inv->vd_ref, inv->vd);
    float32_t iq_ref = pi_update(&inv->vq_pi, inv->vq_ref, inv->vq);

    /* Inner current loop → modulation voltage */
    float32_t vd_out = pi_update(&inv->id_pi, id_ref, inv->id);
    float32_t vq_out = pi_update(&inv->iq_pi, iq_ref, inv->iq);

    /* Apply inverse Park → SVM */
}
```

## 3. Space Vector Modulation (SVM)

### Duty Calculation

```c
/**
 * @brief  Space-vector-equivalent duty computation using common-mode
 *         injection. This avoids sector-specific bookkeeping while
 *         producing centered three-phase duties for a normalized αβ
 *         voltage reference.
 *
 * @param  v_alpha  α-axis voltage reference (normalized to Vdc/2)
 * @param  v_beta   β-axis voltage reference (normalized to Vdc/2)
 * @param  t_a      Output: phase A duty [0..1]
 * @param  t_b      Output: phase B duty [0..1]
 * @param  t_c      Output: phase C duty [0..1]
 */
void svm_calculate(float32_t v_alpha, float32_t v_beta,
                   float32_t *t_a, float32_t *t_b, float32_t *t_c) {
    /* Convert αβ to three-phase references normalized to Vdc/2. */
    float32_t v_a = v_alpha;
    float32_t v_b = -0.5f * v_alpha + 0.8660254f * v_beta;
    float32_t v_c = -0.5f * v_alpha - 0.8660254f * v_beta;

    /* Common-mode injection centers the three phase commands and
       produces the same duty result as centered SVPWM while staying in
       the linear modulation region. */
    float32_t v_max = v_a;
    float32_t v_min = v_a;
    if (v_b > v_max) { v_max = v_b; }
    if (v_c > v_max) { v_max = v_c; }
    if (v_b < v_min) { v_min = v_b; }
    if (v_c < v_min) { v_min = v_c; }

    float32_t v_offset = 0.5f * (v_max + v_min);

    *t_a = 0.5f + 0.5f * (v_a - v_offset);
    *t_b = 0.5f + 0.5f * (v_b - v_offset);
    *t_c = 0.5f + 0.5f * (v_c - v_offset);

    if (*t_a > 1.0f) { *t_a = 1.0f; } else if (*t_a < 0.0f) { *t_a = 0.0f; }
    if (*t_b > 1.0f) { *t_b = 1.0f; } else if (*t_b < 0.0f) { *t_b = 0.0f; }
    if (*t_c > 1.0f) { *t_c = 1.0f; } else if (*t_c < 0.0f) { *t_c = 0.0f; }
}
```

## 4. Anti-Islanding Protection (Grid-Tied)

### Requirements (IEEE 1547)
- **Detection time**: Must detect island condition within 2 seconds
- **Response**: Cease energizing within 2 seconds of island formation
- **Methods**: Passive (voltage/frequency shift), active (impedance perturbation)

```c
/**
 * @brief  Passive anti-islanding: monitor grid voltage and frequency.
 *         Trip if voltage or frequency exceeds limits.
 *
 *         IEEE 1547 Default Limits:
 *         Voltage:  88% < V < 110% of nominal
 *         Frequency: 59.3Hz < f < 60.5Hz (60Hz system)
 */
typedef struct {
    float32_t v_rms;
    float32_t freq_hz;
    float32_t v_nominal;
    float32_t f_nominal;
    uint32_t  trip_counter;      /* Debounce counter               */
    uint32_t  trip_threshold;    /* Cycles to confirm (e.g., 10)   */
    bool      island_detected;
} anti_island_t;

void anti_island_check(anti_island_t *ai) {
    bool v_fault = (ai->v_rms < 0.88f * ai->v_nominal) ||
                   (ai->v_rms > 1.10f * ai->v_nominal);
    bool f_fault = (ai->freq_hz < (ai->f_nominal - 0.7f)) ||
                   (ai->freq_hz > (ai->f_nominal + 0.5f));

    if (v_fault || f_fault) {
        ai->trip_counter++;
        if (ai->trip_counter >= ai->trip_threshold) {
            ai->island_detected = true;
            /* Disable PWM outputs, open contactor */
        }
    } else {
        ai->trip_counter = 0U;
    }
}
```

## 5. Harmonic Compensation

### PR Controllers for Selective Harmonic Elimination

```c
/**
 * @brief  Add PR (proportional-resonant) controllers at harmonic
 *         frequencies to reduce specific current harmonics.
 *
 *         For grid-tied: compensate 5th, 7th, 11th, 13th harmonics
 *         For off-grid with nonlinear loads: 3rd, 5th, 7th harmonics
 *
 * @note   Each PR controller adds computational cost.
 *         Typical budget: 3-4 harmonic compensators maximum in ISR.
 *         Use slow-loop update for harmonic compensator integrators.
 */
typedef struct {
    float32_t kr;           /* Resonant gain                        */
    float32_t omega_h;      /* Harmonic frequency [rad/s]           */
    float32_t s1, s2;       /* State variables                      */
    float32_t bandwidth;    /* Resonant bandwidth [rad/s]           */
} pr_harmonic_t;

#define MAX_HARMONICS  4U

typedef struct {
    pr_harmonic_t harmonic[MAX_HARMONICS];
    uint8_t count;
} harmonic_compensator_t;

/* Initialize for 5th, 7th, 11th harmonics (50Hz grid) */
void harmonic_comp_init(harmonic_compensator_t *hc, float32_t f_grid) {
    const uint8_t orders[] = {5U, 7U, 11U, 13U};
    const float32_t kr_values[] = {10.0f, 8.0f, 5.0f, 3.0f};

    hc->count = MAX_HARMONICS;
    for (uint8_t i = 0U; i < hc->count; i++) {
        hc->harmonic[i].kr = kr_values[i];
        hc->harmonic[i].omega_h = 6.2832f * f_grid * (float32_t)orders[i];
        hc->harmonic[i].bandwidth = 5.0f;  /* rad/s */
        hc->harmonic[i].s1 = 0.0f;
        hc->harmonic[i].s2 = 0.0f;
    }
}
```

## 6. MPPT Integration (Solar Inverter)

### Perturb & Observe Algorithm

```c
/**
 * @brief  MPPT Perturb & Observe for solar inverter.
 *         Adjusts DC bus voltage reference to track maximum power point.
 *         Executes in slow background task (100ms–1s period).
 *
 * @note   MPPT output feeds into grid-tied inverter's Vdc_ref
 *         or directly sets Id_ref in current-source mode.
 */
typedef struct {
    float32_t v_pv;           /* PV voltage [V]                  */
    float32_t i_pv;           /* PV current [A]                  */
    float32_t p_pv;           /* PV power [W]                    */
    float32_t p_prev;         /* Previous power sample [W]       */
    float32_t v_ref;          /* Voltage reference output [V]    */
    float32_t step_size;      /* Perturbation step [V]           */
    int8_t    direction;      /* +1 or -1                        */
} mppt_pno_t;

void mppt_pno_update(mppt_pno_t *mppt) {
    mppt->p_pv = mppt->v_pv * mppt->i_pv;

    float32_t dp = mppt->p_pv - mppt->p_prev;

    if (dp > 0.0f) {
        /* Power increased → keep same direction */
    } else {
        /* Power decreased → reverse direction */
        mppt->direction = -mppt->direction;
    }

    mppt->v_ref += mppt->direction * mppt->step_size;
    mppt->p_prev = mppt->p_pv;
}
```

## 7. Single-Phase Inverter Specifics

### H-Bridge Unipolar/Bipolar PWM
- **Bipolar**: Both legs switch at fsw, simpler but higher output ripple
- **Unipolar**: One leg at fsw, other at line frequency — effective ripple frequency = 2×fsw, smaller filter
- **Hybrid**: Switch modulation strategy based on output voltage level

### Single-Phase dq Control
- **Challenge**: Single-phase system has no natural orthogonal component for Park transform
- **Solution**: Create virtual β-axis using:
  1. Transport delay (T/4 delay)
  2. Second-order generalized integrator (SOGI)
  3. All-pass filter

## 8. Protection Summary

| Fault | Grid-Tied | Off-Grid | Detection |
|-------|-----------|----------|-----------|
| **Grid OVP/UVP** | Disconnect + anti-island | N/A | PLL + RMS |
| **Grid Over/Under Freq** | Disconnect | N/A | PLL frequency |
| **Output OCP** | Reduce Id_ref or trip | Current limit clamp | ADC + comparator |
| **DC Bus OVP** | Stop injecting power | Reduce output | ADC watchdog |
| **Anti-Islanding** | Mandatory | N/A | V/f monitoring |
| **Ground Fault** | Residual current detection | Residual current | Differential CT |

## 9. STM32G4 Implementation Notes

- **CORDIC**: Use for all sin/cos in Park/Clarke transforms and SVM angle calculations
- **HRTIM or TIM1**: Center-aligned PWM with complementary outputs and dead-time
- **Dual ADC**: Simultaneous current sampling for accurate dq transform
- **OPAMP**: Internal opamps for current sensing in each phase leg
- **COMP**: Fast overcurrent protection per phase, linked to HRTIM fault inputs for automatic PWM shutdown
- **DAC**: Generate programmable threshold for COMP-based OCP
