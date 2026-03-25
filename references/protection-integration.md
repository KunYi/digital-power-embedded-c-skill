# Protection Integration (Reference)

## Project Integration Notes

For robust protection in digital power systems, consider these integration patterns:

- **Hierarchical Protection**: Hardware → Software → System level protection with decreasing response time but increasing sophistication
- **Fail-Safe Design**: All protection mechanisms should default to safe state (power off) on any uncertainty
- **Recovery Strategy**: Progressive recovery with back-off timers to prevent rapid fault cycling

## Recommended Protection Structure

```c
typedef struct {
    bool hardware_fault_latched;   // Latched by trip hardware or ISR bridge
    bool software_fault_latched;   // Confirmed by supervisory logic
    bool lockout_latched;          // Requires explicit operator/service clear
    uint32_t fault_flags;          // Bitfield of active or latched faults
    uint32_t trip_counter;         // Re-trip count inside observation window
    uint32_t recovery_timer_ms;    // Back-off timer for auto-reset path
} protection_state_t;
```

## 1. Protection hierarchy
- Hardware fault: overcurrent, overvoltage, hardware trip (fast path)
- Software supervisory: RMS overlimit, gate driver error, auxiliary sensor mismatch
- System safe states: immediate PWM off, gate driver disable, contactor open
- Ownership rule:
  - Hardware trip path owns immediate energy interruption
  - Fast ISR owns latching the event into software-visible state
  - Slow supervisory task owns debounce, retry policy, and lockout escalation

## 2. Integration with control loops
- Control outputs from PI/PR are gated by `enable` and `fault` flags
- On protection trigger: store requested output for later recovery and then clamp output to 0
- Load reduction path: switch from normal vector to safe zero-vector when fault exists
- Reset rule:
  - Hard faults clear controller integrators before any restart attempt
  - Soft supervisory faults may preserve filtered monitor states if restart logic needs them
  - No control loop may resume until PWM ownership has explicitly returned from protection logic

## 3. PWM shutdown path
- Prefer a hardware trip path first: comparator / fault input / timer break should force PWM off without waiting for software
- After the hardware trip, software should latch the fault, clear control states, and place PWM outputs in a known safe state
- If software explicitly disables outputs, set `TIMx->BDTR` MOE=0 (or the equivalent HRTIM fault action) only after confirming the timer-specific shutdown sequence

## 4. Fault recovery / auto reset
- On auto reset request, verify: PFC input voltage in-range, temperature normal, no overcurrent
- Clear PI integrator when switching from fault to run to avoid bump
- If repeated trip in short time, escalate to latched lockout
- Suggested clear policy:
  - Hardware transient trip: auto-clear only after both the hardware source and software latch are clear
  - Recoverable software fault: clear in supervisory task after debounce and restart criteria pass
  - Lockout fault: clear only on explicit operator command or power-cycle, per product policy

## 5. STM32 specifics
- Use BOD/VBAT/ADC comparators for external protection signals
- Use EXTI interrupts for immediate hardware protection lines
- Keep the software follow-up path short and deterministic so it does not delay fault latching, restart logic, or supervision
