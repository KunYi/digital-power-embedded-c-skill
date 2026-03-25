# PI Fast Path (Reference)

## 1. Standard vs Fast-path Comparison

### Standard PI
- Full computation: error calc + P-term + I-term accumulation + anti-windup + saturation
- Cycle count: ~15-20 cycles on STM32G4 FPU
- Suitable for 10kHz loops

### Fast-path PI
- Optimized for hot-path: minimize branches, precompute constants
- Cycle count: ~8-12 cycles
- Suitable for 20-40kHz ISR loops

## 2. Fast-path Implementation Techniques

### Precomputed Coefficients
- Store Ki*Ts as constant in init phase
- Avoid multiplication in ISR: use pre-multiplied values
- Example: `integral += precomputed_ki_ts * error;`

### Branch Elimination
- Use conditional move instead of if-statements
- Clamp output without branches: `output = (output > max) ? max : (output < min) ? min : output;`
- But prefer saturation functions from CMSIS DSP

### State Minimization
- Keep only essential state: integral, previous error (if needed)
- Avoid complex anti-windup in fast path; defer to slow loop

## 3. Extreme Hot-path (ISR Critical)
- Inline all operations, no function calls
- Use register variables for state
- Preload constants into registers
- Example structure:
```c
// In ISR
float error = ref - feedback;
float p_term = kp * error;
integral += ki_ts * error;
float unsaturated = p_term + integral;
output = arm_clip_f32(unsaturated, min, max);
```

## 4. Anti-windup in Fast Path
- Simple clamping: if output saturated, stop integrator growth
- Back-calculation: `integral += kb * (saturated_output - unsaturated_output);`
- Choose Kb = 0.1-1.0 based on system dynamics

## 5. STM32G4 Specific Optimizations
- Use FPU single-precision operations
- Leverage CORDIC if angle computations are involved (though not directly for PI)
- Use `__attribute__((section(".ramfunc")))` for ISR functions if needed
- Profile with STM32CubeIDE debugger to ensure cycle budget

## 6. Integration with Control Loops
- Fast PI in current loop ISR (40kHz)
- Standard PI in voltage loop (10kHz)
- Coordinate integrator reset on protection events
- Ensure consistent scaling across loops
