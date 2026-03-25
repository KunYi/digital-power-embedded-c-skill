# State Machine Recovery Templates (Reference)

## Project Integration Notes

For seamless integration with existing digital power projects, consider these architectural patterns:

- **Layered State Management**: Separate hardware fault detection, software confirmation, and control state transitions
- **Recovery Strategy**: Implement progressive recovery (auto-reset → manual reset → system restart) based on fault severity
- **State Persistence**: Use non-volatile storage for critical fault history and recovery counters

## Recommended State Structure

```c
typedef enum {
    STATE_INIT,
    STATE_READY,
    STATE_RUN,
    STATE_FAULT,
    STATE_RECOVERY,
    STATE_SHUTDOWN
} system_state_t;

typedef enum {
    FAULT_NONE,
    FAULT_OCP,      // Over-current protection
    FAULT_OVP,      // Over-voltage protection
    FAULT_UVP,      // Under-voltage protection
    FAULT_OTP,      // Over-temperature protection
    FAULT_PHASE_LOSS,
    FAULT_COMM_ERROR
} fault_type_t;

typedef enum {
    FAULT_CLASS_TRANSIENT,
    FAULT_CLASS_RECOVERABLE,
    FAULT_CLASS_LOCKOUT
} fault_class_t;

typedef struct {
    system_state_t state;
    fault_type_t active_fault;
    fault_class_t fault_class;
    uint32_t retry_count;
    uint32_t retry_limit;
    bool operator_reset_required;
} recovery_context_t;
```

## 1. Fault lifecycle model
- Detection: raw conditions for OVP/OCP/Overtemp
- Confirmation: debounce, count cycles, confirm with 2nd sensor
- Execution: de-energize switches, set FAULT flag, PWM inhibit
- Recovery: timed/restart condition, toggle relay, clear fault when safe
- Ownership contract:
  - ISR may detect and latch only hard faults plus minimum transition context
  - Main/supervisory state machine decides whether the fault is transient, recoverable, or lockout
  - Restart sequence owns relay timing, integrator reset, and soft-start re-entry

## 2. Substate layers
- `RUN` state has substate `Normal`, `SoftStop`, `SoftStart`
- `FAULT` state can have `Latch`, `AutoReset`, `SystemHold`
- Use explicit enumerated value to avoid implicit truthiness
- Suggested reset semantics:
  - `FAULT/Latch`: wait for explicit clear condition or operator reset
  - `FAULT/AutoReset`: bounded retry count with back-off timer
  - `SystemHold`: maintenance or service mode, no autonomous restart

## 3. State prioritization
- Hard fault (OC/OV) > soft fault (RMS overlimit) > normal control
- In ISR, evaluate only hard-fault logic; soft-fault in 1ms supervision
- Disable integrator when any fault is active

## 4. Recovery chain
- Auto-recovery path: FAULT->HOLD (wait t=500ms) -> verify sensors -> RUN
- Manual reset path: FAULT->RESET on user command
- Keep lockout counter to prevent rapid cycling
- Practical restart contract:
  - Verify fault source is inactive
  - Reinitialize controller states and output clamps
  - Re-arm PWM only after precharge / gate-driver / relay conditions are valid
  - Re-enter `RUN/SoftStart` first, never jump directly back to full-power run

## 5. STM32 implementation hints
- Use `volatile` flags for state and `atomic` accesses in C11 if available
- Keep all state transitions deterministic in ISR and at main loop
- Use debug logging (USART / ITM) only in non-real-time context
