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
- Immediately set update register to 0% for all channels
- Set `TIMx->BDTR` MOE=0 (main output enable) for full hardware disable
- Use `volatile` latch bit to avoid compiler reordering

## 4. Fault recovery / auto reset
- On auto reset request, verify: PFC input voltage in-range, temperature normal, no overcurrent
- Clear PI integrator when switching from fault to run to avoid bump
- If repeated trip in short time, escalate to latched lockout

## 5. STM32 specifics
- Use BOD/VBAT/ADC comparators for external protection signals
- Use EXTI interrupts for immediate hardware protection lines
- Keep software path less than 50us on each pass to avoid watchdog from resetting
