# PI Code Variants (Reference)

## Selection Guide

### Choose Standard PI When
- Project is still in functional debugging phase
- Need to prioritize troubleshooting control logic issues
- ISR budget is still available
- Team has multiple members and needs high readability

### Choose Fast-path PI When
- PI structure has been confirmed correct
- ISR cycle budget is getting tight
- Need to accelerate without significantly sacrificing readability

### Choose Extreme Hot-path PI When
- Bottleneck has been clearly identified in PI itself
- Coefficients and limits are stable
- Team can accept specialized implementation
- There is a clear validation plan

## 1. Standard PI
- Discrete form: u(k) = Kp*e(k) + Ki*Ts*sum(e)
- Code sequence: error calculation -> proportional term -> integrator accumulation -> anti-windup / back-calculation -> output saturation
- Parameters: Kp, Ki, Ts
- Protection: detect input anomalies (NaN/Inf/out-of-range), soft-fault flag
- Maintenance: integrator state monitoring, initialization to 0

## 2. Fast-path PI
- Optimization: flatten branches in ISR execution path
- Constraint: precompute Ki*Ts and 1/(1+Ki*Ts) to avoid divisions
- Key optimization: minimize instruction count, ideally 4~6 multiplies
- Data type: 32-bit floating point

## 3. Extreme hot-path PI
- Hardened pipeline: pre-add, delayed update, minimal register traffic
- Avoid function calls and conditional branches in critical path
- Typical strategy: core path only P+I, state storage/monitoring in lower-priority loop

## 4. Anti-windup / back-calculation
- Formula: u_pi = u_unsat; integrator += Ki*Ts*e + Kb*(u_sat-u_unsat)
- Design note: choose Kb in [0.1, 1] depending on system response and stability
- Under saturation: integrator growth should be restrained

## 5. CMSIS DSP / fixed-point / CORDIC support
- Fixed point: Q31/Q15 scaling and overflow handling
- Data type: int32_t as alternative to float32_t
- Reference implementation: using CMSIS DSP (arm_math.h) or CORDIC for trig operations on STM32G4

## 6. STM32 recommendations
- On STM32G4, use FPU float / float32_t
- Compiler flags: -mcpu=cortex-m4 -mfpu=fpv4-sp-d16 -mfloat-abi=hard
- ISR policy: only minimal control computations in ISR, preferably complete in <1/3 SysTick interval
