# LPF and RMS Variants (Reference)

## 1. First-order LPF
- **Physical Meaning**: Low-pass filter attenuates high-frequency noise while preserving DC/low-frequency components
- **Discrete Form**: y(k) = a*y(k-1) + (1-a)*x(k) where a is the pole location
- **Coefficient Calculation**: a = exp(-2π*fc/fs) where fc=cutoff frequency, fs=sampling frequency
- **Numerical Note**: Use float32_t with FPU for accuracy < 0.1%; fixed-point requires careful scaling

## 2. Multi-stage LPF / RMS
- **Cascade Strategy**: Connect 2-3 first-order filters in series to achieve steeper roll-off and better noise rejection
- **RMS Calculation**: Root Mean Square = √(mean(x²)) - measures signal magnitude regardless of waveform shape
- **Implementation**: Square and accumulate samples in fast ISR, compute sqrt in slower task (1ms/10ms) to save cycles
- **Cascade Example**: x₁ = LPF₁(x), x₂ = LPF₁(x₁), x₃ = LPF₁(x₂) for third-order response

## 3. Coefficient pre-calculation
- **Optimization**: Compute a, (1-a), and derived coefficients during initialization
- **Avoid Runtime Math**: Never use exp(), pow(), or division in ISR - precompute everything
- **Storage**: Use const float arrays for coefficient tables

## 4. Extreme LPF
- **Fractional Delay Approximation**: Use IIR structure instead of floating-point exponential for fixed coefficients
- **Lookup Table**: Precompute filter responses for common cutoff frequencies
- **MAC Optimization**: Minimize multiply-accumulate operations for high-frequency execution

## 5. Fault filtering and system safety
- **Input Validation**: Disable filter output if ADC input is out-of-range or sampling fault detected
- **Fault Decision**: Use slow 2nd-order LPF (τ = 10ms~100ms) for reliable fault confirmation
- **Safety Integration**: Filter outputs should feed protection logic with appropriate time constants

## 6. STM32 recommendations
- Use arm_math_fir_f32 and arm_rms_f32 from CMSIS DSP
- Sync DMA+ADC trigger with TIM1/TIM8 PWM for simultaneous sampling
