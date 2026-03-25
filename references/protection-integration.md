# Protection Integration (Reference)

## Project Integration Notes

For robust protection in digital power systems, consider these integration patterns:

- **Hierarchical Protection**: Hardware → Software → System level protection with decreasing response time but increasing sophistication
- **Fail-Safe Design**: All protection mechanisms should default to safe state (power off) on any uncertainty
- **Recovery Strategy**: Progressive recovery with back-off timers to prevent rapid fault cycling

## Recommended Protection Structure

```c
typedef struct {
    bool hardware_fault;      // Immediate hardware response
    bool software_fault;      // Confirmed software decision
    bool system_fault;        // System-level shutdown
    uint32_t fault_flags;     // Bitfield of active faults
    uint32_t recovery_timer;  // Back-off timer for auto-reset
} protection_state_t;
```

## 1. Protection hierarchy
- Hardware fault: overcurrent, overvoltage, hardware trip (fast path)
- Software supervisory: RMS overlimit, gate driver error, auxiliary sensor mismatch
- System safe states: immediate PWM off, gate driver disable, contactor open

## 2. Integration with control loops
- Control outputs from PI/PR are gated by `enable` and `fault` flags
- On protection trigger: store requested output for later recovery and then clamp output to 0
- Load reduction path: switch from normal vector to safe zero-vector when fault exists

## 3. PWM shutdown path
- Prefer a hardware trip path first: comparator / fault input / timer break should force PWM off without waiting for software
- After the hardware trip, software should latch the fault, clear control states, and place PWM outputs in a known safe state
- If software explicitly disables outputs, set `TIMx->BDTR` MOE=0 (or the equivalent HRTIM fault action) only after confirming the timer-specific shutdown sequence

## 4. Fault recovery / auto reset
- On auto reset request, verify: PFC input voltage in-range, temperature normal, no overcurrent
- Clear PI integrator when switching from fault to run to avoid bump
- If repeated trip in short time, escalate to latched lockout

## 5. STM32 specifics
- Use BOD/VBAT/ADC comparators for external protection signals
- Use EXTI interrupts for immediate hardware protection lines
- Keep the software follow-up path short and deterministic so it does not delay fault latching, restart logic, or supervision
