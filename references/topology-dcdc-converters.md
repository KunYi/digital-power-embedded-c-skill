# Topology: DC-DC Converters (Reference)

## Overview

Buck, boost, and buck-boost converters are the fundamental DC-DC building blocks in digital power systems. Key control challenges include CCM/DCM boundary handling, output voltage regulation dynamics, synchronous rectification timing, and load transient response.

## 1. Buck Converter

### Voltage-Mode Control

```c
/**
 * @brief  Buck converter voltage-mode control with single PI loop.
 *         Suitable for low-power applications with slow load transients.
 *
 * @note   Duty = Vout / Vin in steady state (ideal).
 *         PI compensates for losses, parasitic drops, and load changes.
 */
typedef struct {
    pi_controller_t voltage_pi;
    float32_t v_out;            /* Measured output voltage [V]     */
    float32_t v_out_ref;        /* Output voltage reference [V]    */
    float32_t v_in;             /* Input voltage (for feedforward) */
    float32_t duty;             /* Duty cycle [0..Dmax]            */
    float32_t duty_max;         /* Maximum duty limit              */
    float32_t duty_min;         /* Minimum duty limit              */
} buck_vmode_t;

void buck_voltage_mode_update(buck_vmode_t *bk) {
    float32_t v_error = bk->v_out_ref - bk->v_out;
    bk->duty = pi_update(&bk->voltage_pi, v_error);

    /* Optional: input voltage feedforward for improved line rejection */
    /* bk->duty += bk->v_out_ref / bk->v_in;  (add to PI output) */
}
```

### Peak Current Mode Control (PCMC)

```c
/**
 * @brief  Peak current mode control for buck converter.
 *         Outer voltage loop generates peak current reference.
 *         Inner current limit is implemented via hardware comparator.
 *
 * @note   Slope compensation is MANDATORY for duty > 50% to prevent
 *         subharmonic oscillation. Compensation ramp = Se ≥ 0.5 × Sn
 *         where Sn = (Vin - Vout) × Rsense / L (inductor down-slope).
 */
typedef struct {
    pi_controller_t voltage_pi;
    float32_t i_peak_ref;       /* Peak current reference from voltage loop */
    float32_t slope_comp;       /* Slope compensation [A/cycle]            */
    float32_t v_out;
    float32_t v_out_ref;
} buck_pcmc_t;

void buck_pcmc_voltage_loop(buck_pcmc_t *bk) {
    float32_t v_error = bk->v_out_ref - bk->v_out;
    bk->i_peak_ref = pi_update(&bk->voltage_pi, v_error);
}

/**
 * @brief  Calculate slope compensation ramp value.
 * @param  v_in    Input voltage [V]
 * @param  v_out   Output voltage [V]
 * @param  l_h     Inductance [H]
 * @param  r_sense Current sense resistor [Ω]
 * @param  f_sw    Switching frequency [Hz]
 * @return Slope compensation per cycle [A]
 */
float32_t buck_slope_comp(float32_t v_in, float32_t v_out,
                          float32_t l_h, float32_t r_sense,
                          float32_t f_sw) {
    float32_t sn = (v_in - v_out) * r_sense / l_h;  /* Down-slope     */
    float32_t t_sw = 1.0f / f_sw;
    return 0.5f * sn * t_sw;  /* Minimum compensation = 50% of Sn */
}
```

### Average Current Mode Control

```c
/**
 * @brief  Average current mode: dual-loop control.
 *         Outer: voltage PI → current reference
 *         Inner: current PI → duty cycle
 *
 * @note   More robust than PCMC, no slope compensation needed.
 *         Requires current sensor with DC accuracy (shunt or hall).
 *         Inner loop bandwidth: ~1/10 of switching frequency.
 */
typedef struct {
    pi_controller_t voltage_pi;   /* Outer loop: 1-10 kHz   */
    pi_controller_t current_pi;   /* Inner loop: fsw/10     */
    float32_t v_out;
    float32_t v_out_ref;
    float32_t i_inductor;         /* Average inductor current */
    float32_t i_ref;
    float32_t duty;
} buck_acmc_t;

void buck_acmc_slow_loop(buck_acmc_t *bk) {
    float32_t v_error = bk->v_out_ref - bk->v_out;
    bk->i_ref = pi_update(&bk->voltage_pi, v_error);
}

void buck_acmc_fast_loop(buck_acmc_t *bk) {
    float32_t i_error = bk->i_ref - bk->i_inductor;
    bk->duty = pi_update(&bk->current_pi, i_error);
}
```

## 2. Boost Converter

### Key Differences from Buck
- **Duty equation**: D = 1 - Vin/Vout (steady-state)
- **Right-half-plane zero**: Limits control bandwidth — voltage loop must be slower
- **Inductor current ≠ load current**: Input current equals inductor current, output is pulsating

### RHP Zero Consideration

```c
/**
 * @brief  Boost converter voltage loop bandwidth must be limited
 *         due to right-half-plane zero (RHPZ).
 *
 *         f_rhpz = R_load × (1 - D)² / (2π × L)
 *
 *         Rule of thumb: voltage loop crossover < f_rhpz / 5
 *         Aggressive tuning: crossover < f_rhpz / 3
 *
 * @param  r_load  Load resistance [Ω]
 * @param  duty    Operating duty cycle
 * @param  l_h     Inductance [H]
 * @return RHPZ frequency [Hz]
 */
float32_t boost_calc_rhpz(float32_t r_load, float32_t duty,
                          float32_t l_h) {
    float32_t one_minus_d = 1.0f - duty;
    return (r_load * one_minus_d * one_minus_d) / (6.2832f * l_h);
}
```

## 3. Buck-Boost / Four-Switch Buck-Boost

### Mode Transition

```c
/**
 * @brief  Four-switch buck-boost seamless mode transition.
 *         When Vin ≈ Vout, both buck and boost legs are active.
 *
 * @note   Hysteresis band prevents rapid mode switching:
 *         - Pure buck:  Vin > Vout + margin
 *         - Buck-boost: Vout - margin < Vin < Vout + margin
 *         - Pure boost: Vin < Vout - margin
 */
typedef enum {
    BBMODE_BUCK,
    BBMODE_BUCK_BOOST,
    BBMODE_BOOST
} buckboost_mode_t;

buckboost_mode_t buckboost_detect_mode(float32_t v_in, float32_t v_out,
                                       float32_t hysteresis) {
    if (v_in > (v_out + hysteresis)) {
        return BBMODE_BUCK;
    } else if (v_in < (v_out - hysteresis)) {
        return BBMODE_BOOST;
    } else {
        return BBMODE_BUCK_BOOST;
    }
}
```

## 4. CCM / DCM Boundary

### Detection and Handling

```c
/**
 * @brief  Detect CCM/DCM boundary from inductor current.
 *         In DCM, inductor current reaches zero during off-time.
 *
 * @note   DCM detection methods:
 *         1. Software: check if i_inductor < threshold near end of off-time
 *         2. Hardware: use comparator on current sense to detect zero-crossing
 *         3. Predictive: calculate ripple and compare to average current
 *
 *         In DCM, control plant transfer function changes:
 *         - Additional pole appears
 *         - Gain increases at light load
 *         - May need different PI coefficients
 */
typedef struct {
    bool is_dcm;
    float32_t i_ripple_pp;      /* Peak-to-peak ripple current     */
    float32_t i_boundary;       /* CCM/DCM boundary current        */
} ccm_dcm_state_t;

void ccm_dcm_detect(ccm_dcm_state_t *state, float32_t i_avg,
                     float32_t v_in, float32_t v_out,
                     float32_t l_h, float32_t f_sw) {
    /* Calculate peak-to-peak ripple: ΔI = (Vin × D) / (L × fsw) */
    float32_t duty = v_out / v_in;  /* Buck steady-state */
    state->i_ripple_pp = (v_in * duty) / (l_h * f_sw);

    /* Boundary current = half of ripple (average current where
       valley just touches zero) */
    state->i_boundary = state->i_ripple_pp * 0.5f;

    /* DCM when average current drops below boundary */
    state->is_dcm = (i_avg < state->i_boundary);
}
```

## 5. Synchronous Rectification

- **Dead-time Control**: Prevent shoot-through; typical 50–200 ns on STM32G4 HRTIM
- **Body Diode Conduction**: Minimize by turning on synchronous FET as fast as possible after dead-time
- **Light Load Efficiency**: Disable synchronous rectification in DCM to prevent negative current
- **STM32G4 HRTIM**: Hardware dead-time insertion with programmable rising/falling edge delays

```c
/* HRTIM dead-time configuration example */
void hrtim_deadtime_config(uint16_t rising_ns, uint16_t falling_ns) {
    /* HRTIM clock = 170 MHz × 32 = 5.44 GHz effective (184 ps resolution) */
    uint16_t dt_rising  = (uint16_t)(rising_ns * 1000U / 184U);
    uint16_t dt_falling = (uint16_t)(falling_ns * 1000U / 184U);

    HRTIM1->sTimerxRegs[0].DTxR =
        (dt_rising  << HRTIM_DTR_DTR_Pos) |
        (dt_falling << HRTIM_DTR_DTF_Pos) |
        HRTIM_DTR_DTPRSC_0;  /* Prescaler */
}
```

## 6. Multi-Phase Interleaving

- **Purpose**: Reduce input/output ripple, share thermal load, increase power capability
- **Phase Shift**: 360°/N for N phases (e.g., 2-phase = 180° offset)
- **Current Sharing**: Each phase needs independent current sensing; balance via per-phase PI trimming
- **STM32G4**: Use multiple HRTIM timer units with master trigger for phase synchronization

## 7. Protection Summary

| Fault | Buck | Boost | Notes |
|-------|------|-------|-------|
| **Output OVP** | Monitor Vout | Monitor Vout | Boost: more critical due to energy storage |
| **Input UVP** | Disable switching | Disable switching | Prevent excessive duty at low Vin |
| **OCP** | Cycle-by-cycle (COMP) | Cycle-by-cycle (COMP) | Boost: input = inductor current |
| **Reverse Current** | Disable sync FET | Disable sync FET | Especially at light load |
| **Thermal** | NTC via ADC, 1ms check | NTC via ADC, 1ms check | Derate power above threshold |

## 8. STM32G4 Implementation Notes

- **HRTIM vs TIM1**: Use HRTIM for >100 kHz or when sub-ns dead-time precision is needed
- **COMP + DAC**: Use internal comparators with DAC reference for programmable OCP threshold
- **OPAMP**: Internal opamps for current sense signal conditioning
- **ADC Trigger**: HRTIM ADC trigger units (ADC1R–ADC4R) for precise sampling point placement
