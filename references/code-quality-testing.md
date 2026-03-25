# Code Quality and Testing (Reference)

## 1. MISRA C Compliance Strategy

### Rule Categories and Priorities
- **Required Rules**: Must comply, no deviations allowed
- **Advisory Rules**: Should comply, deviations need justification
- **STM32G4 Specific**: Register access, interrupt handling, floating-point usage

### Common MISRA Violations in Digital Power Code
- **Rule 10.1**: Implicit conversion between float and integer
- **Rule 11.4**: Casting pointer to integer for register access
- **Rule 12.1**: Precedence confusion in complex expressions
- **Rule 13.2**: Testing float for equality

```c
// MISRA-compliant register access
#define ADC1_BASE_ADDR    (0x50040000UL)
#define ADC_ISR_OFFSET    (0x00UL)
#define ADC_ISR_ADDR      (ADC1_BASE_ADDR + ADC_ISR_OFFSET)

static inline uint32_t adc_get_isr(void) {
    return *((volatile uint32_t *)ADC_ISR_ADDR);
}

// MISRA-compliant float comparison
#define FLOAT_EPSILON     (1.0e-6f)

static inline bool float_equal(float32_t a, float32_t b) {
    return (fabsf(a - b) < FLOAT_EPSILON);
}
```

### Deviation Documentation
- **Justification Required**: Each deviation needs technical justification
- **Impact Analysis**: Document safety and functionality impact
- **Review Process**: Deviations reviewed by safety team

## 2. Static Analysis Integration

### Tool Selection for STM32G4
- **PC-Lint/Gimpel Lint**: Comprehensive C analysis
- **Coverity**: Deep flow analysis, good for embedded
- **Cppcheck**: Free, fast, good for basic checks
- **STM32CubeIDE**: Built-in static analysis

### Configuration for Digital Power Code
```xml
<!-- Example cppcheck configuration -->
<rules>
    <rule name="misra-c2012-10.1">style</rule>
    <rule name="misra-c2012-11.4">warning</rule>
    <rule name="unusedVariable">style</rule>
    <rule name="unreadVariable">warning</rule>
</rules>

<paths>
    <include>src/</include>
    <include>include/</include>
    <exclude>tests/</exclude>
</paths>
```

### Automated Analysis in CI/CD
- **Pre-commit Hooks**: Run static analysis before commits
- **Build Integration**: Fail build on critical violations
- **Quality Gates**: Different thresholds for different branches

## 3. Unit Testing Framework

### Unity Test Framework for Embedded
- **Lightweight**: Minimal resource usage, suitable for MCU
- **STM32G4 Integration**: Run tests on target or host simulation
- **Test Organization**: Separate test files, mock hardware interfaces

```c
// Example test file structure
#include "unity.h"
#include "pi_controller.h"

// Mock hardware interfaces
float32_t mock_adc_read(void) {
    return 1.5f;  // Simulated feedback voltage
}

void test_pi_controller_basic(void) {
    pi_controller_t pi;
    pi_init(&pi, 0.1f, 0.01f, 0.0f, 10.0f);

    float32_t output = pi_update(&pi, 2.0f, 1.5f);  // ref=2.0, fb=1.5

    TEST_ASSERT_FLOAT_WITHIN(0.1f, 0.25f, output);  // Expected P-term
}

void test_pi_anti_windup(void) {
    pi_controller_t pi;
    pi_init(&pi, 1.0f, 0.1f, 0.0f, 1.0f);  // Small output limit

    // Drive integrator high
    for (int i = 0; i < 100; i++) {
        pi_update(&pi, 10.0f, 0.0f);  // Large error
    }

    float32_t output = pi_update(&pi, 0.0f, 0.0f);  // Zero error
    TEST_ASSERT_FLOAT_WITHIN(0.1f, 0.0f, output);  // Should not be saturated
}
```

### Hardware Abstraction Layer (HAL) for Testing
- **Interface Definition**: Abstract hardware dependencies
- **Mock Implementation**: Provide simulated hardware for unit tests
- **Conditional Compilation**: Switch between real and mock implementations

```c
// Hardware abstraction interface
typedef struct {
    float32_t (*adc_read)(uint8_t channel);
    void (*pwm_write)(uint8_t channel, float32_t duty);
    uint32_t (*timer_get_count)(void);
} hardware_interface_t;

// Real hardware implementation
static float32_t real_adc_read(uint8_t channel) {
    // STM32G4 ADC reading code
    return (float32_t)ADC1->DR * ADC_SCALE;
}

// Mock implementation for testing
static float32_t mock_adc_read(uint8_t channel) {
    static float32_t mock_values[] = {1.0f, 2.0f, 3.0f};
    return mock_values[channel % 3];
}

// Global hardware interface pointer
hardware_interface_t *hw = &real_hardware;

#ifdef UNIT_TEST
hw = &mock_hardware;
#endif
```

## 4. Integration Testing Strategies

### Hardware-in-the-Loop (HIL) Testing
- **Real-time Simulation**: Plant model runs on host PC
- **STM32G4 Target**: Control code runs on actual hardware
- **Communication**: Ethernet or serial link between PC and target

### Closed-Loop Validation
- **Test Scenarios**: Step response, load transients, fault conditions
- **Performance Metrics**: Settling time, overshoot, steady-state error
- **Safety Validation**: Fault detection and recovery timing

```c
// Integration test framework
typedef struct {
    float32_t reference_voltage;
    float32_t load_current;
    float32_t expected_output;
    uint32_t timeout_ms;
} test_scenario_t;

bool run_integration_test(test_scenario_t *scenario) {
    // Set up test conditions
    set_reference_voltage(scenario->reference_voltage);
    set_load_current(scenario->load_current);

    // Wait for settling
    uint32_t start_time = get_system_time();
    while (get_system_time() - start_time < scenario->timeout_ms) {
        if (check_settling_condition(scenario->expected_output)) {
            return true;
        }
    }
    return false;
}
```

## 5. Code Review Checklist

### Architecture Review
- [ ] Modular design with clear separation of concerns
- [ ] Appropriate use of abstraction layers
- [ ] Configurable parameters for different applications
- [ ] Hardware independence through HAL

### Performance Review
- [ ] ISR execution time within budget
- [ ] Memory usage optimized (stack, heap, globals)
- [ ] Efficient use of STM32G4 hardware features (CORDIC, FPU, DMA)
- [ ] No blocking operations in real-time paths

### Safety and Reliability Review
- [ ] Input validation and bounds checking
- [ ] Fault detection and recovery mechanisms
- [ ] Watchdog integration and heartbeat monitoring
- [ ] MISRA C compliance and deviation justification

### Maintainability Review
- [ ] Clear naming conventions and documentation
- [ ] Consistent code formatting and style
- [ ] Unit tests for critical functions
- [ ] Version control and change tracking

## 6. Documentation Standards

### Doxygen Integration
- **Function Documentation**: Parameters, return values, preconditions
- **File Headers**: Purpose, author, version history
- **Architecture Documentation**: Block diagrams, data flow

```c
/**
 * @brief PI controller update function
 * @param pi Pointer to PI controller structure
 * @param reference Reference input value
 * @param feedback Feedback input value
 * @return Controller output value
 * @pre pi must be initialized with pi_init()
 * @post Output is clamped to configured limits
 */
float32_t pi_update(pi_controller_t *pi, float32_t reference, float32_t feedback);
```

### Requirements Traceability
- **Link Code to Requirements**: Trace matrix from spec to implementation
- **Change Impact Analysis**: Track how changes affect requirements
- **Verification Records**: Document test results against requirements

## 7. Continuous Integration Pipeline

### Automated Build and Test
- **Build Verification**: Compile for all supported configurations
- **Static Analysis**: Run automated code quality checks
- **Unit Test Execution**: Run test suite on every commit
- **Coverage Analysis**: Track code coverage metrics

### Quality Metrics Dashboard
- **Code Complexity**: Cyclomatic complexity, function length
- **Code Coverage**: Statement and branch coverage
- **Defect Density**: Bugs per thousand lines of code
- **Performance Benchmarks**: Execution time, memory usage

## 8. Version Control and Release Management

### Branching Strategy
- **Main Branch**: Production-ready code
- **Development Branch**: Integration branch
- **Feature Branches**: Individual feature development
- **Release Branches**: Stabilization before release

### Semantic Versioning
- **MAJOR.MINOR.PATCH**: Breaking.NewFeature.BugFix
- **Pre-release Labels**: alpha, beta, rc (release candidate)
- **Build Metadata**: Build number, commit hash

### Release Process
- **Code Freeze**: No new features in release branch
- **Regression Testing**: Full test suite execution
- **Documentation Update**: Release notes and user manuals
- **Binary Distribution**: Firmware images with checksums
