# PLL Extreme Variants (Reference)

## Project Alignment Notes

For better integration with existing projects, consider the following layered approach:

- **PLL Algorithm Core**: Generates `sample_valid / freq_hz / quality / angle_step_abs`
- **State Machine Interface Layer**: Consumes lock quality and timing stability
- **Slow Task Layer**: Executes lock confirmation counting and timeout determination

## Recommended Interface Structure

```c
typedef struct {
    uint16_t sample_valid;
    uint16_t phase_continuous_ok;
    uint16_t phase_sequence_ok;
    float32_t freq_hz;
    float32_t angle_step_abs;
    float32_t quality;
} pll_sync_feedback_t;
```

## 1. SRF-PLL (Synchronous Reference Frame PLL)
- Core: Park transform to dq and PI control on q-axis
- Angle update: theta += w_est*Ts
- Bandwidth: choose based on PLL update rate and grid disturbance goals,
  typically far below switching frequency and often in the tens-of-Hz range

## 2. DSOGI-PLL
- Use second-order generalized integrator to reject harmonics
- Performs orthogonal signal generation and quadrature output
- Lock detection by q-axis PI close to zero

## 3. DDSRF / EPLL
- DDSRF: directly generate sine and cosine with DDS, then perform SRF
- EPLL: enhanced with phase extrapolation and rate limiter

## 4. Lock state logic
- States: UNLOCK -> LOCKING -> LOCKED -> HOLD -> UNLOCK
- Criteria: amplitude > threshold, q-error < small tolerance, d-error stability
- Handle unbalanced and distorted grid by reducing bandwidth and adding notch compensation

## 5. STM32 suggestions
- Use FPU or CORDIC for trig operations as appropriate for the platform
- Cycle budget: keep PLL in slow loop (1kHz) or execute in high-priority but with limited ops
- If using DMA + ADC, keep voltage sampling coherent and time-stamped relative to the PLL update period
- Leverage CORDIC for sin/cos calculations in angle computation for better performance
