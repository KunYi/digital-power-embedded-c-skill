# Topology: PFC and Vienna Rectifier (Reference)

## Overview

Power Factor Correction (PFC) and Vienna rectifier topologies convert AC mains to regulated DC bus. The control challenge is simultaneously achieving high power factor (PF > 0.99), low THD (< 5%), and tight DC bus voltage regulation.

## 1. Single-Phase Boost PFC

### Control Architecture

```
                ┌──────────────┐
  Vac ─────────►│  Rectifier   ├──── IL ────┐
                └──────────────┘            │
                                    ┌───────▼───────┐
                                    │  Boost Stage   │──── Vbus (400V)
                                    └───────┬───────┘
                                            │
                ┌───────────────────────────┘
                │
        ┌───────▼────────┐    ┌──────────┐    ┌──────────┐
        │ Current Loop   │◄───┤ Voltage  │◄───┤ Vbus_ref │
        │ (PI, 40kHz)    │    │ Loop (PI,│    │ (400V)   │
        └───────┬────────┘    │ 1-10kHz) │    └──────────┘
                │             └──────────┘
                ▼
             PWM Duty
```

### Dual-Loop Control Strategy

```c
/**
 * @brief  PFC boost dual-loop control.
 *         Outer loop: voltage PI → generates current reference
 *         Inner loop: current PI → generates duty cycle
 *
 * @note   Current reference is shaped as |sin(θ)| from PLL angle
 *         to force input current to follow mains voltage waveform.
 */
typedef struct {
    pi_controller_t voltage_pi;   /* Slow loop: Vbus regulation     */
    pi_controller_t current_pi;   /* Fast loop: inductor current    */
    float32_t v_bus;              /* Measured DC bus voltage [V]    */
    float32_t v_bus_ref;          /* DC bus reference (e.g., 400V) */
    float32_t i_inductor;         /* Measured inductor current [A] */
    float32_t i_ref;              /* Current reference [A]         */
    float32_t duty;               /* Output duty cycle [0..1]      */
    float32_t sin_theta;          /* |sin(θ)| from PLL             */
} pfc_control_t;

/* Voltage loop — executed in slow loop (1–10 kHz) */
void pfc_voltage_loop(pfc_control_t *pfc) {
    float32_t i_amplitude = pi_update(&pfc->voltage_pi,
                                      pfc->v_bus_ref,
                                      pfc->v_bus);

    /* Shape current reference: I_ref = I_amplitude × |sin(θ)| */
    pfc->i_ref = i_amplitude * pfc->sin_theta;
}

/* Current loop — executed in fast ISR (20–40 kHz) */
void pfc_current_loop(pfc_control_t *pfc) {
    pfc->duty = pi_update(&pfc->current_pi, pfc->i_ref, pfc->i_inductor);
}
```

### Feedforward Compensation

```c
/**
 * @brief  Input voltage feedforward improves transient response
 *         and reduces distortion at zero-crossing.
 *         duty = 1 - Vin/Vbus (steady-state boost equation)
 */
float32_t pfc_feedforward(float32_t v_in_rect, float32_t v_bus) {
    if (v_bus > 10.0f) {  /* Avoid division by near-zero */
        return 1.0f - (v_in_rect / v_bus);
    }
    return 0.0f;
}

/* Combined: duty = PI_output + feedforward */
pfc->duty = pi_update(&pfc->current_pi, pfc->i_ref, pfc->i_inductor)
          + pfc_feedforward(v_in_rectified, pfc->v_bus);
```

### Zero-Crossing Distortion Mitigation
- **Problem**: Near AC zero-crossing, inductor current reference approaches zero → PI loses control
- **Solutions**:
  1. Clamp minimum current reference to prevent sign reversal
  2. Disable PI integration near zero-crossing (|sin(θ)| < 0.05)
  3. Use feedforward-dominant control near crossing region

## 2. Three-Phase Vienna Rectifier

### Topology Characteristics
- **3-level operation**: Reduces dv/dt stress and switching losses
- **Bidirectional diode bridge + active switches**: Only 3 switches for 3 phases
- **Split DC bus**: Two output capacitors (Vbus/2 each), requires balance control

### Control Architecture

```c
/**
 * @brief  Vienna rectifier control in dq-frame.
 *         d-axis controls active power (DC bus voltage)
 *         q-axis controls reactive power (typically set to 0)
 *
 * @note   Vienna rectifier constrains switch states differently
 *         from standard 3-phase inverter — SVM must be adapted.
 */
typedef struct {
    pi_controller_t v_bus_pi;      /* DC bus voltage loop          */
    pi_controller_t id_pi;         /* d-axis current loop          */
    pi_controller_t iq_pi;         /* q-axis current loop          */
    pi_controller_t v_balance_pi;  /* Capacitor voltage balance    */
    float32_t id_ref;              /* d-axis current reference [A] */
    float32_t iq_ref;              /* q-axis current reference [A] (typically 0) */
    float32_t v_bus_upper;         /* Upper capacitor voltage [V]  */
    float32_t v_bus_lower;         /* Lower capacitor voltage [V]  */
} vienna_control_t;

/* DC bus voltage balance — prevents capacitor voltage drift */
void vienna_balance_control(vienna_control_t *vc) {
    float32_t v_diff = vc->v_bus_upper - vc->v_bus_lower;
    float32_t balance_offset = pi_update(&vc->v_balance_pi, 0.0f, v_diff);
    /* Inject small offset into d-axis to redistribute power */
    vc->id_ref += balance_offset;
}
```

### Sector Detection for Vienna SVM

```c
/**
 * @brief  Detect voltage sector from three-phase input voltages.
 *         Vienna rectifier switch action depends on current
 *         direction AND voltage polarity in each phase.
 * @return Sector number (1–6)
 */
uint8_t vienna_detect_sector(float32_t va, float32_t vb, float32_t vc) {
    uint8_t sector = 0U;
    if (va > 0.0f) { sector |= 0x01U; }
    if (vb > 0.0f) { sector |= 0x02U; }
    if (vc > 0.0f) { sector |= 0x04U; }

    /* Map bit pattern to sector number */
    static const uint8_t sector_map[8] = {0, 1, 3, 2, 5, 6, 4, 0};
    return sector_map[sector];
}
```

## 3. THD Optimization Strategies

- **Current Loop Bandwidth**: ≥ 2 kHz for < 5% THD; higher bandwidth tracks harmonics better
- **Harmonic Compensation**: Add PR controllers tuned to 3rd, 5th, 7th harmonics
- **Dead-time Compensation**: Pre-calculate dead-time effect and inject correction voltage
- **ADC Timing**: Sample inductor current at PWM valley to capture true average

## 4. Protection Specific to PFC/Vienna

| Fault | Detection Method | Response |
|-------|-----------------|----------|
| **Bus OVP** | ADC + analog watchdog, threshold ~420V | Immediate PWM off, discharge resistor |
| **Bus UVP** | Software check in 1ms task | Prevent startup, disable load |
| **Inrush OCP** | Hardware comparator, ~2× rated | Stop switching, wait then soft-restart |
| **Phase Loss** | PLL unlock + voltage asymmetry | Reduce power or shutdown |
| **Bus Imbalance** | |V_upper - V_lower| > threshold | Balance PI corrects; fault if > 10% |

## 5. STM32G4 Implementation Notes

- **HRTIM**: Use for high-resolution PWM (184 ps) in interleaved PFC designs
- **CORDIC**: Compute sin(θ) for current shaping directly from PLL angle
- **OPAMP**: Internal opamps for current sensing amplification (no external amplifier needed)
- **COMP**: Fast analog comparators for cycle-by-cycle OCP (< 100 ns response)
- **Dual ADC**: ADC1 for current, ADC2 for voltage — simultaneous sampling for accurate power calculation
