# dq / Clarke / Park Transform Optimization (Reference)

## 1. Data path conventions
- **Clarke Transform**: Converts three-phase (a,b,c) to two-phase (α,β) stationary reference frame
  - α = ia (direct axis component)
  - β = (ia + 2*ib)/√3 for balanced three-phase with i_a, i_b, i_c implied
  - Physical meaning: α represents the real axis, β represents the imaginary axis in stationary frame
- **Park Transform**: Rotates stationary frame to synchronous rotating frame (d,q)
  - d = α*cos(θ) + β*sin(θ) (direct axis - aligns with grid voltage vector)
  - q = -α*sin(θ) + β*cos(θ) (quadrature axis - 90° phase shift)
  - Physical meaning: d-axis tracks active power, q-axis tracks reactive power
- **Inverse Transform**: Converts back from rotating frame to stationary frame
  - α = d*cos(θ) - q*sin(θ)
  - β = d*sin(θ) + q*cos(θ)
  - Used for generating three-phase PWM references

## 2. Angle source and sync
- Use PLL angle `theta` from grid tracking
- Keep in same units (radians) and avoid incremental drift
- If fast ISR and slow PLL mistiming, use linear interpolation between PLL updates

## 3. Normalization and sequence
- For positive sequence, define phasors positive in abc order
- For negative sequence, use reversed sign for q
- Keep transformation scaling constant: 2/3 or sqrt(2/3) depending on magnitude definition

## 4. Optimization techniques
- Precompute cos(theta) and sin(theta) once for both current and voltage transforms
- Use blended sine table when direct CORDIC is expensive
- Minimize repeated multiplies by sharing intermediate terms
- Use `arm_sin_f32` / `arm_cos_f32` in plus/minus the same angle with CORDIC acceleration on STM32G4

## 5. STM32 guidance
- For high-speed path, use fixed-precision with `arm_` operations only in non-ISR updates
- Keep full transform in 10~20 cycle budget by state reusing
- Evaluate CPU load and if needed move to 2x ISR period at 20kHz -> 10kHz
