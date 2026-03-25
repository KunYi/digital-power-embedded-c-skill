# Topology: LLC and PSFB Resonant Converters (Reference)

## Overview

LLC half-bridge and Phase-Shifted Full-Bridge (PSFB) are isolated DC-DC converters widely used for high-efficiency bus-to-load conversion. Both topologies achieve Zero-Voltage Switching (ZVS) to minimize switching losses. The primary control variable differs: LLC uses **switching frequency**, PSFB uses **phase-shift angle**.

## 1. LLC Resonant Converter

### Operating Principle
- **Resonant tank**: Lr (series inductor) + Cr (series capacitor) + Lm (magnetizing inductor)
- **Two resonant frequencies**:
  - fr1 = 1/(2π√(Lr×Cr)) — series resonance (full load)
  - fr2 = 1/(2π√((Lr+Lm)×Cr)) — parallel resonance (no load)
- **Operating range**: fr2 < fsw < fr1 for ZVS turn-on across load range
- **Gain characteristic**: Voltage gain = f(fsw, Q, Ln) — nonlinear, load-dependent

### Frequency Control

```c
/**
 * @brief  LLC frequency-mode control.
 *         Voltage loop PI output maps to switching frequency.
 *
 * @note   LLC gain is inversely related to frequency near resonance:
 *         - Increase fsw → decrease gain → decrease Vout
 *         - Decrease fsw → increase gain → increase Vout
 *         This is OPPOSITE to duty-cycle control — PI polarity must
 *         be inverted or reference/feedback swapped.
 *
 *         Frequency limits are critical:
 *         - f_min: below this, ZVS is lost → hard switching, damage
 *         - f_max: above this, gain too low → cannot regulate
 */
typedef struct {
    pi_controller_t voltage_pi;
    float32_t v_out;
    float32_t v_out_ref;
    float32_t freq_hz;          /* Current switching frequency [Hz]  */
    float32_t freq_min;         /* Minimum frequency limit [Hz]      */
    float32_t freq_max;         /* Maximum frequency limit [Hz]      */
    float32_t freq_resonant;    /* Series resonant frequency fr1 [Hz]*/
} llc_control_t;

void llc_voltage_loop(llc_control_t *llc) {
    /* Note: reference/feedback are intentionally reversed because LLC gain
       decreases with increasing frequency. */
    float32_t freq_cmd = pi_update(&llc->voltage_pi,
                                   llc->v_out,
                                   llc->v_out_ref);

    /* Clamp to safe operating range */
    if (freq_cmd < llc->freq_min) { freq_cmd = llc->freq_min; }
    if (freq_cmd > llc->freq_max) { freq_cmd = llc->freq_max; }

    llc->freq_hz = freq_cmd;
}
```

### Frequency-to-Timer Conversion

```c
/**
 * @brief  Convert target frequency to HRTIM period register value.
 * @param  freq_hz   Target switching frequency [Hz]
 * @param  hrtim_clk HRTIM clock frequency [Hz] (e.g., 170MHz × 32)
 * @return Timer period register value
 */
uint32_t llc_freq_to_period(float32_t freq_hz, uint32_t hrtim_clk) {
    return (uint32_t)((float32_t)hrtim_clk / freq_hz);
}

/* Update HRTIM period and keep 50% duty (symmetric half-bridge) */
void llc_update_pwm(llc_control_t *llc) {
    uint32_t period = llc_freq_to_period(llc->freq_hz, HRTIM_EFFECTIVE_CLK);
    HRTIM1->sTimerxRegs[0].PERxR = period;
    HRTIM1->sTimerxRegs[0].CMP1xR = period / 2U;  /* 50% duty */
}
```

### Burst Mode for Light Load

```c
/**
 * @brief  LLC burst mode control for high efficiency at light load.
 *         When load drops below threshold, stop switching and coast.
 *         Resume when output drops below hysteresis band.
 *
 * @note   Burst mode prevents excessive frequency increase at light load
 *         which would cause: high switching losses, audible noise,
 *         and potential loss of ZVS.
 */
typedef enum {
    LLC_BURST_ACTIVE,     /* Switching normally         */
    LLC_BURST_IDLE        /* Switches off, coasting     */
} llc_burst_state_t;

void llc_burst_control(llc_control_t *llc, llc_burst_state_t *state) {
    const float32_t burst_enter_threshold = 0.2f;  /* V above ref */
    const float32_t burst_exit_threshold  = 0.1f;  /* V below ref */

    if (*state == LLC_BURST_ACTIVE) {
        if (llc->v_out > (llc->v_out_ref + burst_enter_threshold)) {
            *state = LLC_BURST_IDLE;
            /* Disable PWM outputs */
        }
    } else {  /* LLC_BURST_IDLE */
        if (llc->v_out < (llc->v_out_ref - burst_exit_threshold)) {
            *state = LLC_BURST_ACTIVE;
            /* Re-enable PWM, start at f_max for soft engagement */
            llc->freq_hz = llc->freq_max;
        }
    }
}
```

### ZVS Verification
- **Method**: Monitor switch node voltage to confirm zero-crossing before turn-on
- **Hardware**: Use comparator on switch node or monitor dead-time current direction
- **Loss of ZVS indicators**: f_sw < f_min, light load without burst mode, abnormal resonant tank

## 2. Phase-Shifted Full-Bridge (PSFB)

### Operating Principle
- **Full-bridge topology**: 4 switches (2 legs), secondary rectification
- **Phase-shift control**: Leading leg and lagging leg have phase offset 0°–180°
- **Effective duty**: D_eff = phase_shift / 180° (determines power transfer)
- **ZVS mechanism**: Resonant transition using leakage inductance + parasitic capacitance

### Phase-Shift Control

```c
/**
 * @brief  PSFB phase-shift control.
 *         Voltage PI output maps to phase-shift angle between legs.
 *
 * @note   Unlike LLC, PSFB behaves like a buck converter through
 *         the transformer: increasing phase-shift increases Vout.
 *         PI polarity is same as standard buck duty-cycle control.
 */
typedef struct {
    pi_controller_t voltage_pi;
    pi_controller_t current_pi;   /* Optional: for current-mode */
    float32_t v_out;
    float32_t v_out_ref;
    float32_t i_primary;          /* Primary current [A]           */
    float32_t phase_shift;        /* Phase shift [0.0 .. 1.0]      */
    float32_t phase_max;          /* Maximum phase shift limit      */
} psfb_control_t;

void psfb_voltage_loop(psfb_control_t *psfb) {
    psfb->phase_shift = pi_update(&psfb->voltage_pi,
                                  psfb->v_out_ref,
                                  psfb->v_out);

    /* Clamp phase shift */
    if (psfb->phase_shift > psfb->phase_max) {
        psfb->phase_shift = psfb->phase_max;
    }
    if (psfb->phase_shift < 0.0f) {
        psfb->phase_shift = 0.0f;
    }
}

/* Convert phase-shift to HRTIM compare register */
void psfb_update_pwm(psfb_control_t *psfb, uint32_t period) {
    /* Leading leg: fixed 50% duty */
    HRTIM1->sTimerxRegs[0].CMP1xR = period / 2U;

    /* Lagging leg: delayed by phase_shift × period */
    uint32_t delay = (uint32_t)(psfb->phase_shift * (float32_t)period);
    HRTIM1->sTimerxRegs[1].CMP1xR = delay + (period / 2U);
}
```

### Duty Cycle Loss Compensation

```c
/**
 * @brief  PSFB effective duty loss due to commutation interval.
 *         Actual power transfer duty is less than commanded phase shift.
 *
 *         D_loss = (Lleak × I_load) / (Vin × T_sw)
 *
 *         Compensation: add D_loss to PI output as feedforward.
 *
 * @param  l_leak   Leakage inductance [H]
 * @param  i_load   Load current reflected to primary [A]
 * @param  v_in     Input voltage [V]
 * @param  f_sw     Switching frequency [Hz]
 * @return Duty loss fraction
 */
float32_t psfb_duty_loss(float32_t l_leak, float32_t i_load,
                         float32_t v_in, float32_t f_sw) {
    return (l_leak * i_load * f_sw) / v_in;
}
```

### ZVS Conditions for PSFB
- **Leading leg**: Easier to achieve ZVS (full load current assists transition)
- **Lagging leg**: Difficult at light load (only leakage inductance available)
- **Solutions**: Add external inductor, use resonant capacitor, or accept partial hard-switching
- **Design rule**: Lagging leg ZVS ≥ 20% load for acceptable efficiency curve

## 3. Soft-Start Strategies for Isolated Converters

```c
/**
 * @brief  Soft-start for LLC: ramp frequency from f_max down to target.
 *         Starting at high frequency ensures minimum gain → gentle start.
 *
 *         For PSFB: ramp phase-shift from 0 up to target.
 *         Both approaches gradually increase power delivery.
 */
typedef struct {
    float32_t ramp_value;       /* Current ramp position           */
    float32_t ramp_target;      /* Final target value              */
    float32_t ramp_step;        /* Increment per control cycle     */
    bool ramp_complete;
} soft_start_t;

void soft_start_update(soft_start_t *ss) {
    if (!ss->ramp_complete) {
        ss->ramp_value += ss->ramp_step;
        if (ss->ramp_value >= ss->ramp_target) {
            ss->ramp_value = ss->ramp_target;
            ss->ramp_complete = true;
        }
    }
}

/* LLC: start at f_max, ramp down; ramp_step is negative */
/* PSFB: start at 0, ramp up; ramp_step is positive      */
```

## 4. Transformer Saturation Prevention

- **DC Bias Detection**: Monitor magnetizing current asymmetry via primary current sampling
- **Flux Balancing**: Use DC-blocking capacitor in series with primary (LLC inherently has Cr)
- **Software Correction**: Measure and equalize positive/negative volt-second product
- **Startup Risk**: Most dangerous period — use symmetric soft-start to avoid DC bias

## 5. Protection Specific to Isolated Converters

| Fault | LLC | PSFB | Detection |
|-------|-----|------|-----------|
| **Secondary OVP** | Stop switching | Zero phase shift | ADC + analog watchdog |
| **Primary OCP** | Increase frequency | Reduce phase shift | Hardware comparator |
| **ZVS Loss** | Raise f_min | Accept or add inductor | Switch node monitoring |
| **Transformer Sat.** | Shutdown + reset | Shutdown + reset | Primary current asymmetry |
| **Output Short** | Hiccup mode | Hiccup mode | OCP + retry timer |

## 6. STM32G4 Implementation Notes

- **HRTIM**: Essential for LLC variable-frequency control — period register update enables frequency modulation per switching cycle
- **HRTIM Dual Channel**: Use Timer A for leading leg, Timer B for lagging leg (PSFB), master timer for synchronization
- **Dead-time**: HRTIM provides independent rising/falling dead-time for each output — critical for ZVS optimization
- **ADC Trigger**: Use HRTIM ADC trigger to sample primary current at mid-point of conduction interval
- **Burst Mode**: HRTIM output disable/enable with state retention for seamless burst mode implementation
