# Filter and Resonant Variants (Reference)

## 1. PR Controller
- **Physical Meaning**: Proportional-Resonant controller provides infinite gain at specific frequency (grid frequency) for zero steady-state error in AC systems
- **Discrete Implementation**: Second-order resonant term at grid frequency ω₀
- **Typical Transfer Function**: G_pr(z) = Kp + Kr * z⁻¹*(1+z⁻¹)/(1-2cos(ω₀Ts)z⁻¹+z⁻²)
- **Execution Strategy**: Keep P-term in fast ISR, update resonant integrator in slow loop to save cycles
- **Anti-windup**: Clamp resonant output and apply back-calculation: integrator += Kb*(clamped_output - unsaturated_output)

## 2. NOTCH / IIR 2P2Z
- **NOTCH Filter Physical Meaning**: Band-stop filter that attenuates specific frequency components (e.g., harmonics, switching ripple)
- **IIR 2P2Z Structure**: Two poles, two zeros biquad filter with state variables x1,x2,y1,y2 and coefficients a1,a2,b0,b1,b2
- **Discrete Difference Equation**: y(k) = b0*x(k) + b1*x(k-1) + b2*x(k-2) - a1*y(k-1) - a2*y(k-2)
- **Coefficient Management**: Precompute coefficients for nominal sample rate, store in const arrays

## 3. Resonant filter optimization
- **Trigonometric Optimization**: Use cos(2θ) = 2cos²θ - 1 to reduce multiplications for fixed ω₀
- **Complex Vector Format**: For fixed-frequency resonator, represent as complex exponential to eliminate runtime cos/sin calls
- **State Variable Physical Meaning**: s1/s2 represent filter memory, correspond to integrator states in continuous domain

## 4. Implementation notes
- **State Management**: Keep filter state as struct {float s1, s2} for easy reset and predictable sampling order
- **Numerical Stability**: Use Direct Form II Transposed if memory is tight, but monitor quantization effects
- **Overflow Protection**: Detect and saturate gracefully in each update step

## 5. STM32 points
- CMSIS: arm_biquad_cascade_df2T_f32 for 2P2Z
- Use FPU in loop: set -mfpu=fpv4-sp-d16 -mfloat-abi=hard
- Avoid branch-heavy fault handling in 16kHz ISR; defer checks to 1ms task.
