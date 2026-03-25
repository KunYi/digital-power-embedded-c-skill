# Performance Analysis and Optimization (Reference)

## Evidence Levels For Numeric Claims

When quoting timing, throughput, or latency numbers in digital power code
reviews or implementations, distinguish clearly between:

- **Hardware-documented value**: taken from MCU or core documentation
- **Instruction-level estimate**: derived from Arm instruction timing tables,
  usually assuming ideal memory behavior and excluding compiler effects
- **Target measurement**: captured on the real firmware build and hardware

Do not present instruction-level estimates as if they were measured ISR
execution times.

## 1. Real-Time Performance Metrics

### Cycle Budget Analysis
- **Critical Path Identification**: Measure ISR execution time vs available budget
- **STM32G4 Baseline**: 40kHz ISR = 25μs total, leave 10-15μs margin for jitter
- **Measurement Tools**: ARM DWT cycle counter, STM32CubeIDE profiler

```c
// Cycle counting for performance analysis
#define DWT_CONTROL (*((volatile uint32_t*)0xE0001000))
#define DWT_CYCCNT (*((volatile uint32_t*)0xE0001004))

void start_cycle_count(void) {
    DWT_CONTROL |= 1;  // Enable cycle counter
    DWT_CYCCNT = 0;    // Reset counter
}

uint32_t stop_cycle_count(void) {
    return DWT_CYCCNT;
}

// Usage in ISR
void TIM1_UP_IRQHandler(void) {
    start_cycle_count();

    // Control code here

    uint32_t cycles = stop_cycle_count();
    if (cycles > MAX_ISR_CYCLES) {
        fault_detected = true;
    }
}
```

### Memory Bandwidth Analysis
- **Key Metrics**: Data movement vs computation ratio
- **STM32G4 Memory**: 256KB SRAM, optimize for cache efficiency
- **DMA Usage**: Offload data movement to reduce CPU load

## 2. Algorithm Optimization Strategies

### Fixed-Point vs Floating-Point Tradeoffs
- **Floating-Point Advantages**: Easier development, automatic scaling, FPU acceleration
- **Fixed-Point Advantages**: Deterministic execution, no rounding errors, smaller code size
- **STM32G4 Decision**: `float32_t` is a reasonable default for many STM32G4
  projects; switch to Q15/Q31 only when measurement, code size, or determinism
  requirements justify it

### Lookup Table Optimization
- **When to Use**: Trigonometric functions, nonlinear mappings, calibration curves
- **Memory vs Speed Tradeoff**: Precompute vs runtime calculation
- **STM32G4**: CORDIC often faster than LUT for trigonometric functions

```c
// Example: Precomputed sine table for PLL
#define SINE_TABLE_SIZE 1024
const float32_t sine_table[SINE_TABLE_SIZE] = {
    // Precomputed values using: sin(2*PI*i/SINE_TABLE_SIZE)
};

float32_t fast_sin(float32_t angle_rad) {
    // Normalize angle to 0-2π
    float32_t normalized = fmodf(angle_rad, 2*M_PI);
    if (normalized < 0) normalized += 2*M_PI;

    // Convert to table index
    uint32_t index = (uint32_t)(normalized * SINE_TABLE_SIZE / (2*M_PI));
    return sine_table[index % SINE_TABLE_SIZE];
}
```

### Loop Unrolling and SIMD
- **Manual Unrolling**: Reduce loop overhead for small fixed-size loops
- **Instruction-Level Optimization**: Cortex-M4 can benefit from careful inlining and data locality, but it does not provide general-purpose SIMD acceleration for `float32_t`
- **Code Size Impact**: Balance performance vs flash usage

## 3. Memory Layout Optimization

### Data Structure Alignment
- **Alignment**: Align DMA buffers and frequently accessed structures to natural word boundaries; only apply stricter alignment when a measured benefit exists
- **DMA Requirements**: Verify alignment and access width requirements against the specific peripheral and memory path in use
- **Stack Usage**: Minimize stack depth in ISRs

```c
// Optimized structure alignment
typedef struct __attribute__((aligned(32))) {
    float32_t adc_buffer[128];    // ADC samples
    float32_t filter_state[4];    // IIR filter states
    uint32_t timestamp;           // Synchronization
    uint16_t status_flags;        // Control flags
} control_context_t;

// DMA buffer alignment
__attribute__((aligned(32))) float32_t dma_adc_buffer[ADC_BUFFER_SIZE];
```

### Variable Placement Strategy
- **Register Variables**: Use `register` keyword for hot variables in ISRs
- **Global vs Local**: Globals for state persistence, locals for temporaries
- **Const Placement**: Constants in flash, computed values in RAM

## 4. Compiler Optimization Techniques

### GCC Optimization Flags
- **STM32G4 Starting Point**: `-O2` with target FPU flags is a reasonable baseline; add options such as `-ffast-math` only after reviewing numerical side effects
- **Debug vs Release**: `-O0` for debugging, `-O2/-O3` for performance
- **Floating Point**: `-mfloat-abi=hard -mfpu=fpv4-sp-d16`

### Linker Optimizations
- **Section Placement**: Place frequently used code in faster memory regions
- **Dead Code Elimination**: Use `--gc-sections` to remove unused functions
- **RAM Functions**: Consider placing critical ISRs in RAM only after
  measuring whether it materially improves worst-case latency

```c
// Linker script snippet for RAM functions
.ramfunc : {
    . = ALIGN(4);
    *(.ramfunc)
    *(.ramfunc*)
    . = ALIGN(4);
} >RAM
```

### Profile-Guided Optimization
- **Method**: Compile with profiling, then recompile with profile data
- **STM32G4 Application**: Identify hot paths in control algorithms
- **Tools**: gprof, STM32CubeIDE profiler

## 5. Hardware Acceleration Utilization

### CORDIC Advanced Usage
- **Batch Operations**: Process multiple angles in sequence
- **Precision vs Speed**: Choose appropriate CORDIC precision
- **Integration**: Combine with FPU for complex calculations

```c
// STM32G4 CORDIC uses a fixed-point interface.
// Precision = number of iterations (1~15), higher = more accurate but slower.
// Exact latency and precision depend on device configuration and surrounding
// register access overhead, so profile on target hardware.

// Convert radian angle to Q1.31 CORDIC input: q31 = angle_rad / π
// Output is two sequential reads: first cos, then sin (both Q1.31)

#define CORDIC_Q31_SCALE  (2147483648.0f)  /* 2^31 */
#define CORDIC_INV_PI     (1.0f / 3.14159265358979f)

void cordic_sin_cos(float32_t angle_rad, float32_t* sin_val, float32_t* cos_val) {
    // Configure CORDIC:
    //   FUNC   = 0 (Cosine, outputs cos then sin)
    //   PRECISION = 6 (6 iterations, good tradeoff for control loops)
    //   SCALE  = 0 (no input scaling)
    //   NARGS  = 0 (single 32-bit argument: angle only, modulus defaults to 1.0)
    //   NRES   = 1 (two 32-bit results: cos and sin)
    //   ARGSIZE/RESSIZE = 0 (32-bit)
    CORDIC->CSR = (0U << CORDIC_CSR_FUNC_Pos)      |  /* Cosine function   */
                  (6U << CORDIC_CSR_PRECISION_Pos)  |  /* 6 iterations      */
                  (0U << CORDIC_CSR_SCALE_Pos)       |  /* No input scaling  */
                  CORDIC_CSR_NRES;                      /* 2 results (cos+sin) */

    // Convert angle from radians to Q1.31: value = angle_rad / π, clamped to [-1, +1)
    float32_t normalized = angle_rad * CORDIC_INV_PI;
    int32_t q31_angle = (int32_t)(normalized * CORDIC_Q31_SCALE);

    // Write angle — CORDIC starts computation automatically
    CORDIC->WDATA = (uint32_t)q31_angle;

    // Read results: first read = cos, second read = sin (both Q1.31)
    int32_t cos_q31 = (int32_t)CORDIC->RDATA;
    int32_t sin_q31 = (int32_t)CORDIC->RDATA;

    // Convert Q1.31 back to float32_t
    *cos_val = (float32_t)cos_q31 / CORDIC_Q31_SCALE;
    *sin_val = (float32_t)sin_q31 / CORDIC_Q31_SCALE;
}
```

### DMA Channel Optimization
- **Channel Priority**: Assign high priority to control-related DMA
- **Circular Mode**: Use for continuous ADC/PWM data streams
- **Burst Mode**: Transfer multiple samples efficiently

### ADC Oversampling
- **Hardware Averaging**: Use ADC oversampling to reduce CPU filtering load
- **Resolution Tradeoff**: 16-bit effective resolution with hardware averaging
- **STM32G4**: Up to 256x oversampling supported

## 6. Power Consumption Optimization

### Dynamic Frequency Scaling
- **Adaptive CPU Frequency**: Reduce clock during light load periods
- **STM32G4**: Use voltage scaling with frequency changes
- **Control Impact**: Ensure control loop stability across frequency changes

### Peripheral Clock Gating
- **Automatic Gating**: Enable/disable peripheral clocks as needed
- **STM32G4**: Use RCC register control for fine-grained power management
- **Wake-up Latency**: Consider ISR response time requirements

## 7. Debugging and Profiling Tools

### STM32CubeIDE Integration
- **Live Watch**: Monitor variables without stopping execution
- **Statistical Profiling**: Identify performance bottlenecks
- **Trace Features**: Use ETM for detailed execution analysis

### Custom Instrumentation
- **Performance Counters**: Track ISR execution time, missed deadlines
- **Event Logging**: Circular buffer for system events and timing
- **Health Monitoring**: Continuous validation of control loop performance

```c
// Performance monitoring structure
typedef struct {
    uint32_t isr_execution_time;
    uint32_t max_isr_time;
    uint32_t missed_deadlines;
    uint32_t control_error_count;
    float32_t average_cpu_load;
} performance_stats_t;

performance_stats_t perf_stats;

void update_performance_stats(uint32_t execution_cycles) {
    perf_stats.isr_execution_time = execution_cycles;
    if (execution_cycles > perf_stats.max_isr_time) {
        perf_stats.max_isr_time = execution_cycles;
    }
    // Additional monitoring logic
}
```

## 8. Validation and Testing Strategies

### Automated Test Framework
- **Unit Tests**: Test individual functions with mock hardware
- **Integration Tests**: Full system testing with real hardware
- **Performance Tests**: Validate timing and resource usage

### Formal Verification
- **Static Analysis**: Use tools like Coverity or Polyspace
- **Model Checking**: Verify critical properties mathematically
- **STM32G4 Tools**: STM32CubeMX pinout validation, clock tree analysis
