# Architecture Design Patterns (Reference)

## 1. Data Flow Architecture

### Event-Driven Control Loop
- **Pattern**: Central event dispatcher routes ADC/PWM/timer events to appropriate handlers
- **Benefits**: Decouples event sources from processing logic, easier testing and maintenance
- **STM32G4 Implementation**: Use NVIC interrupt priorities and event flags for synchronization

```c
typedef enum {
    EVENT_ADC_COMPLETE,
    EVENT_PWM_UPDATE,
    EVENT_FAULT_DETECTED,
    EVENT_CONTROL_UPDATE
} system_event_t;

typedef struct {
    system_event_t type;
    uint32_t timestamp;
    void* data;
} event_message_t;

// Event queue for inter-task communication
#define EVENT_QUEUE_SIZE 16
event_message_t event_queue[EVENT_QUEUE_SIZE];
volatile uint8_t event_head, event_tail;
```

### Signal Processing Pipeline
- **Pattern**: Chain processing blocks (ADC → Filter → Controller → PWM) with clear interfaces
- **Benefits**: Modular design, easy to insert/remove processing stages, testable components
- **Implementation**: Function pointer arrays or state machine for pipeline execution

```c
typedef struct {
    float32_t (*process)(void* self, float32_t input);
    void* context;
    bool enabled;
} processing_block_t;

#define MAX_PIPELINE_STAGES 8
processing_block_t control_pipeline[MAX_PIPELINE_STAGES];
```

## 2. Configuration-Driven Design

### Parameter Table Approach
- **Pattern**: All control parameters stored in structured tables loaded at startup
- **Benefits**: Runtime reconfiguration, parameter validation, non-volatile storage integration
- **STM32G4 Optimization**: Use flash sectors for parameter storage with wear leveling

```c
typedef struct {
    float32_t kp, ki, kd;
    float32_t output_min, output_max;
    uint16_t sample_rate_hz;
} pi_config_t;

typedef struct {
    pi_config_t current_loop;
    pi_config_t voltage_loop;
    pll_config_t grid_sync;
    filter_config_t harmonics;
} system_config_t;

const system_config_t default_config = {
    .current_loop = {.kp = 0.1f, .ki = 0.01f, .sample_rate_hz = 40000},
    // ... other defaults
};
```

### Calibration and Adaptation
- **Pattern**: Online parameter adaptation based on operating conditions
- **Benefits**: Automatic optimization, temperature compensation, aging adaptation
- **Safety**: Parameter limits and rate limiting to prevent instability

## 3. State-Based Control Architecture

### Hierarchical State Machine
- **Pattern**: System state machine with sub-states for different operating modes
- **Benefits**: Clear state transitions, predictable behavior, easy debugging
- **Implementation**: State function pointers with entry/exit actions

```c
typedef void (*state_handler_t)(system_state_t* context);

typedef struct {
    system_state_t current_state;
    state_handler_t state_handlers[STATE_COUNT];
    uint32_t state_entry_time;
    fault_status_t active_faults;
} system_controller_t;

void state_init(system_controller_t* ctrl) {
    ctrl->current_state = STATE_INIT;
    ctrl->state_handlers[STATE_INIT] = handle_init;
    ctrl->state_handlers[STATE_READY] = handle_ready;
    // ...
}
```

### Mode-Based Parameter Switching
- **Pattern**: Different parameter sets for different operating modes (startup, normal, fault)
- **Benefits**: Optimized performance for each mode, smooth transitions
- **STM32G4**: Use profile switching with minimal latency

## 4. Memory and Performance Optimization

### Static Memory Allocation
- **Pattern**: All memory allocated at compile time, no heap usage
- **Benefits**: Deterministic execution, no fragmentation, MISRA compliance
- **Implementation**: Global structures with fixed sizes

```c
#define MAX_ADC_BUFFERS 4
#define ADC_BUFFER_SIZE 128

static float32_t adc_buffers[MAX_ADC_BUFFERS][ADC_BUFFER_SIZE];
static control_state_t control_states[CONTROL_LOOP_COUNT];
static filter_state_t filter_states[FILTER_COUNT];
```

### RAM Function Placement
- **Pattern**: Critical ISR functions placed in RAM for faster execution
- **Benefits**: Reduced latency, more deterministic timing
- **STM32G4**: Use `__attribute__((section(".ramfunc")))` and linker script configuration

```c
__attribute__((section(".ramfunc")))
void fast_control_isr(void) {
    // Critical control calculations here
    adc_sample = read_adc();
    error = reference - adc_sample;
    output = pi_controller_update(&pi_state, error);
    update_pwm(output);
}
```

## 5. Safety and Reliability Patterns

### Triple Modular Redundancy (TMR)
- **Pattern**: Three independent calculations with voting for critical functions
- **Benefits**: Fault tolerance, continued operation despite single failures
- **Resource Usage**: 3x computation, 2x memory

### Watchdog and Heartbeat Monitoring
- **Pattern**: Independent watchdog monitors all control loops
- **Benefits**: Detects hangs, resets system safely
- **STM32G4**: Use independent watchdog (IWDG) with windowed operation

```c
void control_task(void) {
    // Main control work
    update_control_loops();

    // Feed watchdog
    IWDG->KR = IWDG_KEY_RELOAD;

    // Update heartbeat
    heartbeat_counter++;
}
```

## 6. Testing and Validation Patterns

### Offline Simulation Framework
- **Pattern**: Same control code runs on PC for testing with simulated plant
- **Benefits**: Early validation, automated testing, parameter tuning
- **Implementation**: Conditional compilation for PC vs embedded targets

```c
#ifdef SIMULATION_BUILD
#define ADC_READ() simulated_adc_value
#define PWM_UPDATE(x) simulated_plant_input = x
#else
#define ADC_READ() (ADC1->DR)
#define PWM_UPDATE(x) (TIM1->CCR1 = x)
#endif
```

### Built-in Self-Test (BIST)
- **Pattern**: Automatic testing of hardware and software components at startup
- **Benefits**: Early fault detection, confidence in system integrity
- **Coverage**: ADC, PWM, memory, control loops

## 7. STM32G4 Specific Optimizations

### CORDIC Integration Pattern
- **Pattern**: Hardware CORDIC used for all trigonometric operations
- **Benefits**: 10-50x speedup vs software, deterministic latency
- **Usage**: PLL angle calculation, dq transforms, resonant controllers

### DMA-Driven Data Flow
- **Pattern**: DMA handles data movement, CPU focuses on computation
- **Benefits**: Reduced CPU load, more time for control algorithms
- **STM32G4**: ADC → DMA → memory, memory → DMA → PWM registers

### HRTIM Advanced Features
- **Pattern**: High-resolution timer for precise PWM generation
- **Benefits**: Better THD, reduced switching losses
- **Usage**: Multi-phase PWM, dead time insertion, ADC triggering
