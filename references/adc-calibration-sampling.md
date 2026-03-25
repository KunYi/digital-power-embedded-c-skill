# ADC Calibration and Sampling (Reference)

## Project Integration Notes

For digital power control on STM32G4, ADC sampling accuracy and timing directly determine control loop quality. Consider these integration patterns:

- **Sampling–PWM Synchronization**: ADC trigger must be phase-locked to PWM switching for noise-free current/voltage measurement
- **Calibration at Startup**: Run hardware self-calibration before enabling control loops to minimize offset/gain errors
- **Multi-Channel Sequencing**: Order channels by priority (current sensing → voltage → temperature) and minimize inter-channel delay

## 1. STM32G4 ADC Architecture Overview

### Hardware Features
- **ADC1–ADC5**: Up to 5 independent 12-bit SAR ADCs with hardware oversampling up to 256×
- **Conversion Speed**: 4.27 Msps (single ADC), or dual-ADC interleaved for higher throughput
- **Resolution**: 12-bit native, up to 16-bit effective with hardware oversampling and right-shift
- **Differential/Single-ended**: Configurable per channel, differential mode for precision current sensing
- **Internal Channels**: V_REFINT, V_BAT, temperature sensor, OPAMP outputs

### Key Registers
- `ADCx->CFGR`: Main configuration (resolution, alignment, DMA, oversampling)
- `ADCx->SMPR1/SMPR2`: Sample time selection per channel (2.5 to 640.5 ADC clock cycles)
- `ADCx->SQR1–SQR4`: Regular sequence registers (channel order and length)
- `ADCx->JSQR`: Injected sequence register (hardware-triggered, highest priority)
- `ADCx->OFR1–OFR4`: Per-channel offset registers for hardware offset compensation

## 2. ADC Calibration Strategy

### Hardware Self-Calibration (Mandatory at Startup)

```c
/**
 * @brief  Run ADC hardware self-calibration for both single-ended and
 *         differential modes. Must be called before ADC enable.
 * @param  hadc  Pointer to ADC handle
 * @pre    ADC must be disabled (ADEN = 0)
 * @post   Calibration factors stored in ADCx->CALFACT register
 */
void adc_run_calibration(ADC_HandleTypeDef *hadc) {
    /* Single-ended calibration */
    HAL_ADCEx_Calibration_Start(hadc, ADC_SINGLE_ENDED);

    /* Differential calibration (if differential channels are used) */
    HAL_ADCEx_Calibration_Start(hadc, ADC_DIFFERENTIAL_ENDED);

    /* Optional: store calibration factors for warm-restart */
    uint32_t cal_factor = HAL_ADCEx_Calibration_GetValue(
        hadc, ADC_SINGLE_ENDED);
    (void)cal_factor;  /* Save to NVM if needed */
}
```

### Software Offset Compensation

```c
/**
 * @brief  Measure and compensate ADC zero-offset at startup.
 *         Sample with input shorted or at known reference voltage.
 * @param  hadc     ADC handle
 * @param  channel  Channel to calibrate
 * @return Measured offset in raw ADC counts
 */
int16_t adc_measure_offset(ADC_HandleTypeDef *hadc, uint32_t channel) {
    #define OFFSET_SAMPLES  64U

    int32_t accumulator = 0;
    for (uint32_t i = 0U; i < OFFSET_SAMPLES; i++) {
        HAL_ADC_Start(hadc);
        HAL_ADC_PollForConversion(hadc, 10U);
        accumulator += (int32_t)HAL_ADC_GetValue(hadc);
    }
    return (int16_t)(accumulator / (int32_t)OFFSET_SAMPLES);
}
```

### Hardware Offset Register (OFR) — Runtime Compensation

```c
/**
 * @brief  Use STM32G4 hardware offset register for per-channel
 *         automatic offset subtraction. No CPU overhead in ISR.
 * @note   Offset is subtracted by hardware before data register read.
 *         Result can be signed (SATEN bit) for bipolar measurements.
 */
void adc_set_hw_offset(ADC_TypeDef *adc, uint32_t ofr_index,
                       uint32_t channel, uint16_t offset_value,
                       bool enable_signed) {
    uint32_t ofr = ADC_OFR1_OFFSET1_EN
                 | ((channel & 0x1FU) << ADC_OFR1_OFFSET1_CH_Pos)
                 | (offset_value & 0x0FFFU);

    if (enable_signed) {
        ofr |= ADC_OFR1_SATEN;  /* Enable signed saturation output */
    }

    /* Write to OFR1–OFR4 based on index */
    volatile uint32_t *ofr_reg = &adc->OFR1 + ofr_index;
    *ofr_reg = ofr;
}
```

### Gain Calibration

- **Method**: Apply known reference voltage, measure ADC reading, compute gain factor
- **Formula**: `gain_correction = V_ref_expected / V_ref_measured`
- **Application**: Multiply raw reading by gain correction in slow loop (1ms), not in fast ISR
- **Temperature Drift**: Re-calibrate if temperature changes > 20°C from initial calibration

## 3. PWM–ADC Synchronization

### Trigger Source Configuration

In digital power control, ADC must sample at the correct instant within the PWM cycle:

```
    PWM Counter (Center-Aligned)
    ────────────────────────────
         /\          /\
        /  \        /  \
       /    \      /    \
      /      \    /      \
    ──        ──          ──
              ↑
         ADC Trigger
     (counter = 0, valley)

    Sampling at valley:  average current in continuous mode
    Sampling at peak:    use for peak current sensing
    Dual sampling:       sample at both peak and valley for ripple measurement
```

### Implementation

```c
/**
 * @brief  Configure TIM1 to trigger ADC at PWM counter valley (center-aligned)
 *         for synchronous current sampling.
 *
 * @note   TIM1_TRGO2 is used as ADC external trigger source.
 *         ADC injected group is preferred for deterministic timing.
 */
void adc_pwm_sync_init(void) {
    /* TIM1: center-aligned mode, generate TRGO2 on update event (UEV) */
    TIM1->CR1 |= TIM_CR1_CMS_0;  /* Center-aligned mode 1 */

    /* Output TRGO2 = update event for ADC trigger */
    TIM1->CR2 &= ~TIM_CR2_MMS2;
    TIM1->CR2 |= TIM_CR2_MMS2_1;  /* MMS2 = 010: Update event */

    /* ADC1: Injected group triggered by TIM1_TRGO2 */
    ADC1->JSQR = (ADC_EXTERNALTRIGINJEC_T1_TRGO2 << ADC_JSQR_JEXTSEL_Pos)
               | (ADC_EXTERNALTRIGINJECCONV_EDGE_RISING << ADC_JSQR_JEXTEN_Pos)
               | (0U << ADC_JSQR_JL_Pos)          /* 1 conversion */
               | (ADC_CHANNEL_1 << ADC_JSQR_JSQ1_Pos); /* Current sense ch */
}
```

### Injected vs Regular Group

| Feature | Injected Group | Regular Group |
|---------|---------------|---------------|
| **Priority** | High (preempts regular) | Normal |
| **Trigger** | Hardware (TIM event) | Software or hardware |
| **Max Channels** | 4 | 16 |
| **DMA Support** | No (read JDRx directly) | Yes |
| **Use Case** | Current/voltage in ISR | Temperature, diagnostics |
| **Latency** | Deterministic | May be delayed |

**Recommendation**: Use **injected group** for control-critical signals (inductor current, bus voltage), **regular group + DMA** for monitoring signals (temperature, auxiliary voltages).

## 4. Multi-Channel Sampling Strategies

### Sequential Sampling (Single ADC)

```c
/* Injected sequence: 3 channels in priority order */
/* Ch1: Inductor current, Ch2: Bus voltage, Ch3: Output voltage */
ADC1->JSQR = (2U << ADC_JSQR_JL_Pos)   /* 3 conversions (JL = N-1) */
           | (ADC_CHANNEL_1 << ADC_JSQR_JSQ1_Pos)
           | (ADC_CHANNEL_2 << ADC_JSQR_JSQ2_Pos)
           | (ADC_CHANNEL_3 << ADC_JSQR_JSQ3_Pos)
           | (ADC_EXTERNALTRIGINJECCONV_EDGE_RISING << ADC_JSQR_JEXTEN_Pos)
           | (ADC_EXTERNALTRIGINJEC_T1_TRGO2 << ADC_JSQR_JEXTSEL_Pos);
```

### Simultaneous Dual-ADC Sampling

For applications requiring simultaneous sampling of voltage and current:

```c
/**
 * @brief  Configure ADC1 + ADC2 in dual simultaneous injected mode.
 *         ADC1 samples current (Ch1), ADC2 samples voltage (Ch3)
 *         at the exact same instant triggered by TIM1.
 *
 * @note   Simultaneous sampling eliminates phase error between
 *         current and voltage measurements — critical for accurate
 *         power factor and dq-transform calculations.
 */
void adc_dual_simultaneous_init(void) {
    /* ADC1 as master, ADC2 as slave */
    /* Dual mode: combined injected simultaneous */
    ADC12_COMMON->CCR &= ~ADC_CCR_DUAL;
    ADC12_COMMON->CCR |= ADC_CCR_DUAL_0 | ADC_CCR_DUAL_2;
    /* DUAL = 00101: Combined regular simultaneous + injected simultaneous */

    /* ADC1 injected: current sense channel */
    ADC1->JSQR = (0U << ADC_JSQR_JL_Pos)
               | (ADC_CHANNEL_1 << ADC_JSQR_JSQ1_Pos)
               | (ADC_EXTERNALTRIGINJECCONV_EDGE_RISING << ADC_JSQR_JEXTEN_Pos)
               | (ADC_EXTERNALTRIGINJEC_T1_TRGO2 << ADC_JSQR_JEXTSEL_Pos);

    /* ADC2 injected: voltage sense channel (slave, no trigger config needed) */
    ADC2->JSQR = (0U << ADC_JSQR_JL_Pos)
               | (ADC_CHANNEL_3 << ADC_JSQR_JSQ1_Pos);
}
```

### DMA Double-Buffer for Background Channels

```c
/**
 * @brief  Regular group + DMA circular mode for monitoring channels.
 *         CPU-free continuous sampling of temperature, auxiliary voltages.
 *         Background task reads from buffer without ISR overhead.
 */
#define MONITOR_CHANNELS  4U
#define MONITOR_DEPTH     8U  /* Averaging depth */

static volatile uint16_t adc_monitor_buf[MONITOR_CHANNELS * MONITOR_DEPTH]
    __attribute__((aligned(32)));

void adc_monitor_dma_init(ADC_HandleTypeDef *hadc, DMA_HandleTypeDef *hdma) {
    /* Configure regular sequence: Temp, Vref, Aux1, Aux2 */
    /* Start DMA in circular mode */
    HAL_ADC_Start_DMA(hadc, (uint32_t *)adc_monitor_buf,
                      MONITOR_CHANNELS * MONITOR_DEPTH);
}

/**
 * @brief  Read averaged monitor value. Called from 1ms background task.
 * @param  channel_index  Channel index (0..MONITOR_CHANNELS-1)
 * @return Averaged ADC value
 */
uint16_t adc_monitor_read_avg(uint8_t channel_index) {
    uint32_t sum = 0U;
    for (uint32_t i = 0U; i < MONITOR_DEPTH; i++) {
        sum += adc_monitor_buf[i * MONITOR_CHANNELS + channel_index];
    }
    return (uint16_t)(sum / MONITOR_DEPTH);
}
```

## 5. Hardware Oversampling

### When to Use
- **Noise Reduction**: Increase effective resolution without software filtering CPU cost
- **Resolution Enhancement**: 12-bit → 16-bit effective with 256× oversampling + 4-bit right shift
- **Tradeoff**: Increases conversion time proportionally

```c
/**
 * @brief  Configure hardware oversampling: 16× with 4-bit right-shift
 *         for 16-bit effective resolution (12 + 4 = 16 bits).
 *
 * @note   Total conversion time = 16 × single conversion time.
 *         Suitable for slow-loop voltage measurements, NOT for fast
 *         current loop where single-shot timing is critical.
 */
void adc_oversampling_config(ADC_TypeDef *adc) {
    adc->CFGR2 = ADC_CFGR2_ROVSE           /* Enable regular oversampling  */
               | (3U << ADC_CFGR2_OVSR_Pos) /* 16× oversampling (OVSR=3: 2^(3+1)=16) */
               | (4U << ADC_CFGR2_OVSS_Pos); /* Right-shift 4 bits         */
    /* Note: OVSR encoding on STM32G4: 0=2×, 1=4×, 2=8×, 3=16×,
       4=32×, 5=64×, 6=128×, 7=256× */
}
```

## 6. Sample Time Selection

### Guidelines for Digital Power

| Signal Type | Recommended Sample Time | Rationale |
|-------------|------------------------|-----------|
| **Current (shunt)** | 6.5–12.5 ADC cycles | Low impedance source, fast settling |
| **Current (hall/CT)** | 12.5–24.5 ADC cycles | Higher source impedance |
| **Bus Voltage (divider)** | 24.5–47.5 ADC cycles | High-impedance divider needs settling |
| **Temperature (NTC)** | 47.5–247.5 ADC cycles | Very high impedance, not time-critical |
| **Internal Vrefint** | 247.5 ADC cycles | Per datasheet requirement |

```c
/* Sample time configuration example */
/* Ch1 (current): 12.5 cycles = fast, Ch5 (voltage): 47.5 cycles */
ADC1->SMPR1 = (ADC_SAMPLETIME_12CYCLES_5 << ADC_SMPR1_SMP1_Pos)
            | (ADC_SAMPLETIME_47CYCLES_5 << ADC_SMPR1_SMP5_Pos);
```

### Total Conversion Time Calculation
- **Formula**: `T_conv = (sample_time + 12.5) × T_ADC_CLK`
- **Example**: At ADC_CLK = 42.5 MHz, 12.5-cycle sample time:
  `T_conv = (12.5 + 12.5) / 42.5 MHz = 588 ns`
- **Budget**: For 40 kHz ISR (25 μs period), 3 sequential injected conversions ≈ 1.8 μs → ~7% of budget

## 7. ADC Reading in Control ISR

### Recommended ISR Pattern

```c
/**
 * @brief  ADC injected conversion complete ISR.
 *         Triggered by TIM1 PWM sync → ADC injected group → JEOS interrupt.
 *
 * @note   Execution order: read ADC → scale → control → update PWM
 *         Total budget: < 10 μs for 40 kHz switching
 */
void ADC1_2_IRQHandler(void) {
    /* [1] Clear interrupt flag immediately */
    ADC1->ISR = ADC_ISR_JEOS;

    /* [2] Read injected data registers (hardware offset already applied) */
    int32_t raw_current = (int32_t)ADC1->JDR1;  /* Already offset-compensated */
    int32_t raw_vbus    = (int32_t)ADC1->JDR2;
    int32_t raw_vout    = (int32_t)ADC1->JDR3;

    /* [3] Scale to engineering units (precomputed coefficients) */
    /* current_A = (raw - offset) * scale_factor                 */
    /* scale_factor = V_ref / (4096 * R_shunt * opamp_gain)      */
    float32_t i_inductor = (float32_t)raw_current * adc_scale.current;
    float32_t v_bus      = (float32_t)raw_vbus    * adc_scale.voltage_bus;
    float32_t v_out      = (float32_t)raw_vout    * adc_scale.voltage_out;

    /* [4] Feed to control loop */
    control_loop_update(i_inductor, v_bus, v_out);

    /* [5] Update PWM duty cycle */
    TIM1->CCR1 = (uint32_t)control_output.duty_counts;
}
```

### Scaling Coefficient Precomputation

```c
/**
 * @brief  ADC scaling factors — precomputed at init, used in ISR.
 *         Converts raw 12-bit ADC count to engineering units.
 */
typedef struct {
    float32_t current;      /* A/count: Vref / (4096 * Rshunt * gain) */
    float32_t voltage_bus;  /* V/count: Vref * divider_ratio / 4096   */
    float32_t voltage_out;  /* V/count: Vref * divider_ratio / 4096   */
} adc_scale_t;

static adc_scale_t adc_scale;

void adc_scale_init(void) {
    const float32_t vref = 3.3f;
    const float32_t adc_max = 4096.0f;

    /* Current: 10mΩ shunt, 20× opamp gain → 0.2 V/A */
    adc_scale.current = vref / (adc_max * 0.01f * 20.0f);

    /* Bus voltage: 100k/1k divider → ratio = 101 */
    adc_scale.voltage_bus = (vref * 101.0f) / adc_max;

    /* Output voltage: 10k/1k divider → ratio = 11 */
    adc_scale.voltage_out = (vref * 11.0f) / adc_max;
}
```

## 8. Fault Detection via ADC

### Window Comparator (Analog Watchdog)

```c
/**
 * @brief  Configure ADC analog watchdog for hardware OVP/OCP detection.
 *         Generates interrupt when reading exceeds threshold — faster
 *         than software comparison, guaranteed sub-microsecond response.
 *
 * @param  adc       ADC peripheral
 * @param  channel   Channel to monitor
 * @param  high_thr  Upper threshold (12-bit)
 * @param  low_thr   Lower threshold (12-bit)
 */
void adc_watchdog_config(ADC_TypeDef *adc, uint32_t channel,
                         uint16_t high_thr, uint16_t low_thr) {
    /* AWD1: single channel monitoring */
    adc->CFGR |= ADC_CFGR_AWD1SGL     /* Monitor single channel */
              |  ADC_CFGR_JAWD1EN;     /* Enable on injected group */

    adc->CFGR &= ~ADC_CFGR_AWD1CH;
    adc->CFGR |= (channel << ADC_CFGR_AWD1CH_Pos);

    /* Set thresholds */
    adc->TR1 = ((uint32_t)high_thr << ADC_TR1_HT1_Pos)
             | ((uint32_t)low_thr  << ADC_TR1_LT1_Pos);

    /* Enable AWD1 interrupt */
    adc->IER |= ADC_IER_AWD1IE;
}
```

### Integration with Protection System
- **Hardware Watchdog → ISR**: Sub-μs response for OCP/OVP
- **Software Check → 1ms Task**: RMS over-limit, temperature trending
- **Coordination**: AWD triggers immediate PWM shutdown (TIMx BDTR BIF), software handles recovery decision

## 9. Common Pitfalls

> [!WARNING]
> **Critical mistakes in ADC configuration for digital power:**

1. **Forgetting calibration**: Uncalibrated ADC has ±5 LSB offset — causes steady-state current error
2. **Wrong sample time**: Too short for high-impedance source → inaccurate readings, noise
3. **Regular group for control signals**: May be preempted or delayed — use injected group
4. **Sampling at PWM edge**: Captures switching noise — always sample at valley or peak center
5. **Blocking poll in ISR**: `HAL_ADC_PollForConversion()` in ISR wastes cycles — use interrupt/DMA
6. **Missing volatile on DMA buffers**: Compiler may cache stale values — mark buffers `volatile`
7. **Ignoring inter-channel crosstalk**: Fast channel followed by slow channel can have charge injection — add settling time or reorder sequence
