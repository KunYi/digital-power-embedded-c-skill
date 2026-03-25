# Notch and IIR 2P2Z (Reference)

## 1. NOTCH filter
- **Physical Meaning**: Band-stop filter that attenuates a specific frequency (e.g., switching harmonics, grid harmonics, or mechanical resonances)
- **Discrete Implementation**: y[k] = x[k] - 2*cos(ω₀Ts)*x[k-1] + x[k-2] + feedforward terms
- **Design Parameters**: Center frequency ω₀, bandwidth, and notch depth
- **Implementation Options**: Use Q-format for fixed-point or `arm_biquad_cascade_df2T_f32` for floating-point

## 2. IIR 2P2Z implementation
- **Structure Choice**: Transposed Direct Form II provides low memory usage and good numerical stability
- **State Variables**: s1, s2 represent filter memory (integrator states in continuous domain)
- **Computation Sequence**:
  - y₀ = b₀*x₀ + s₁  (current output)
  - s₁ = b₁*x₀ - a₁*y₀ + s₂  (update first state)
  - s₂ = b₂*x₀ - a₂*y₀  (update second state)
- **Critical States**: s1 and s2 must be cleared/reset on system faults to prevent wind-up

## 3. Dual-biquad chain
- **Higher-Order Filters**: Cascade multiple biquads for sharper frequency response
- **Normalization**: Scale coefficients to prevent overflow in fixed-point implementations
- **Phase Response**: Monitor total phase delay for control loop stability

## 4. Discrete coefficient establishment
- **Design Method**: Start with continuous-time transfer function H(s), apply bilinear transform
- **Parameters**: Natural frequency ωₙ, damping ratio ζ, quality factor Q
- **Precomputation**: Calculate coefficients in float during init, convert to Q31 for DSP if needed

## 5. STM32 usage
- Prefer `arm_biquad_cascade_df2T_f32` for FPU path
- For Q31, `arm_biquad_cascade_df2T_q31` with proper block size
- Synchronize with control loops and update coefficients outside of fast ISR
