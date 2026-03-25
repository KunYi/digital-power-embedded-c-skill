# Engineering Practices and DevOps (Reference)

## 1. Development Environment Setup

### STM32CubeIDE Configuration
- **Workspace Organization**: Separate projects for different board variants
- **Code Templates**: Standardized file headers and function templates
- **Debug Configurations**: Multiple debug setups for different scenarios

```c
// Standard file header template
/**
 * @file control_loop.c
 * @brief Digital power control loop implementation
 * @author [Developer Name]
 * @date [Date]
 * @version 1.0.0
 *
 * This file implements the main control loops for digital power conversion.
 * Supports buck, boost, and buck-boost topologies with PI control.
 */

/* Includes ------------------------------------------------------------------*/
#include "control_loop.h"
#include "pi_controller.h"
#include "adc_driver.h"
#include "pwm_driver.h"

/* Private defines -----------------------------------------------------------*/
#define CONTROL_LOOP_FREQUENCY_HZ    40000U
#define MAX_DUTY_CYCLE               0.95f
#define MIN_DUTY_CYCLE               0.05f

/* Private typedef -----------------------------------------------------------*/
// ... type definitions

/* Private variables ---------------------------------------------------------*/
// ... global variables

/* Private function prototypes -----------------------------------------------*/
// ... function declarations

/* Private functions ---------------------------------------------------------*/
// ... implementation

/* Public functions ----------------------------------------------------------*/
// ... API functions
```

### Version Control Integration
- **Git Hooks**: Pre-commit checks for code style and basic tests
- **Branch Protection**: Require reviews and CI passes for main branch
- **Issue Tracking**: Link commits to issue tracking system

## 2. Build System and Automation

### Makefile Structure for STM32G4
```makefile
# STM32G4 Makefile template
PROJECT_NAME = digital_power_control
BUILD_DIR = build
SRC_DIRS = src src/control src/drivers src/utils
INC_DIRS = include include/control include/drivers

# Toolchain
CC = arm-none-eabi-gcc
OBJCOPY = arm-none-eabi-objcopy
SIZE = arm-none-eabi-size

# MCU specific flags
MCU_FLAGS = -mcpu=cortex-m4 -mthumb -mfpu=fpv4-sp-d16 -mfloat-abi=hard
OPT_FLAGS = -O2 -ffast-math -funroll-loops
WARNING_FLAGS = -Wall -Wextra -Wpedantic -Wshadow -Wconversion

CFLAGS = $(MCU_FLAGS) $(OPT_FLAGS) $(WARNING_FLAGS) \
         $(addprefix -I, $(INC_DIRS)) \
         -DSTM32G474xx \
         -DUSE_HAL_DRIVER

LDFLAGS = $(MCU_FLAGS) \
          -TSTM32G474RETx_FLASH.ld \
          -Wl,--gc-sections \
          -Wl,--print-memory-usage

# Source files
SRCS = $(foreach dir, $(SRC_DIRS), $(wildcard $(dir)/*.c))
OBJS = $(SRCS:%.c=$(BUILD_DIR)/%.o)

$(BUILD_DIR)/$(PROJECT_NAME).elf: $(OBJS)
	$(CC) $(OBJS) $(LDFLAGS) -o $@

$(BUILD_DIR)/%.o: %.c
	@mkdir -p $(dir $@)
	$(CC) $(CFLAGS) -c $< -o $@

.PHONY: clean flash debug
clean:
	rm -rf $(BUILD_DIR)

flash: $(BUILD_DIR)/$(PROJECT_NAME).elf
	openocd -f interface/stlink.cfg -f target/stm32g4x.cfg \
	        -c "program $(BUILD_DIR)/$(PROJECT_NAME).elf verify reset exit"

debug: $(BUILD_DIR)/$(PROJECT_NAME).elf
	arm-none-eabi-gdb $(BUILD_DIR)/$(PROJECT_NAME).elf \
	                  -ex "target extended-remote localhost:3333"
```

### CMake Integration
- **Cross-platform Build**: Support multiple toolchains
- **Dependency Management**: Handle external libraries
- **Test Integration**: Build and run unit tests

```cmake
cmake_minimum_required(VERSION 3.20)
project(digital_power_control VERSION 1.0.0 LANGUAGES C ASM)

# MCU specific configuration
set(MCU_FAMILY STM32G4)
set(MCU_MODEL STM32G474RE)
set(CPU cortex-m4)
set(FPU fpv4-sp-d16)

# Toolchain settings
set(CMAKE_SYSTEM_NAME Generic)
set(CMAKE_SYSTEM_PROCESSOR arm)

set(CMAKE_C_COMPILER arm-none-eabi-gcc)
set(CMAKE_ASM_COMPILER arm-none-eabi-gcc)
set(CMAKE_OBJCOPY arm-none-eabi-objcopy)

# Compiler flags
add_compile_options(
    -mcpu=${CPU}
    -mthumb
    -mfpu=${FPU}
    -mfloat-abi=hard
    -O2
    -ffast-math
    -Wall
    -Wextra
)

# Linker flags
add_link_options(
    -mcpu=${CPU}
    -mthumb
    -mfpu=${FPU}
    -mfloat-abi=hard
    -T${CMAKE_SOURCE_DIR}/STM32G474RETx_FLASH.ld
    -Wl,--gc-sections
    -Wl,--print-memory-usage
)

# Source files
file(GLOB_RECURSE SOURCES
    "src/*.c"
    "startup/*.s"
)

# Include directories
include_directories(
    include
    drivers/STM32G4xx_HAL_Driver/Inc
    drivers/CMSIS/Device/ST/STM32G4xx/Include
    drivers/CMSIS/Include
)

# Create executable
add_executable(${PROJECT_NAME} ${SOURCES})

# Post-build commands
add_custom_command(TARGET ${PROJECT_NAME} POST_BUILD
    COMMAND ${CMAKE_OBJCOPY} -O ihex $<TARGET_FILE:${PROJECT_NAME}> ${PROJECT_NAME}.hex
    COMMAND ${CMAKE_OBJCOPY} -O binary $<TARGET_FILE:${PROJECT_NAME}> ${PROJECT_NAME}.bin
)
```

## 3. Continuous Integration Pipeline

### GitHub Actions for STM32G4
```yaml
name: STM32G4 CI

on:
  push:
    branches: [ main, develop ]
  pull_request:
    branches: [ main ]

jobs:
  build:
    runs-on: ubuntu-latest

    steps:
    - uses: actions/checkout@v3

    - name: Setup ARM GCC
      uses: carlosperate/arm-none-eabi-gcc-action@v1
      with:
        release: '11.2-2022.02'

    - name: Install dependencies
      run: |
        sudo apt-get update
        sudo apt-get install -y cmake ninja-build

    - name: Build project
      run: |
        mkdir build
        cd build
        cmake -G Ninja ..
        ninja

    - name: Run unit tests
      run: |
        cd build
        ctest --output-on-failure

    - name: Static analysis
      run: |
        cppcheck --enable=all --std=c99 --language=c \
                 --suppress=missingIncludeSystem \
                 src/ include/

    - name: Upload artifacts
      uses: actions/upload-artifact@v3
      with:
        name: firmware
        path: |
          build/*.elf
          build/*.hex
          build/*.bin
```

### Quality Gates
- **Build Success**: All compilation targets must succeed
- **Test Pass Rate**: Minimum 90% unit test pass rate
- **Static Analysis**: Zero critical violations
- **Code Coverage**: Minimum 80% statement coverage

## 4. Documentation Generation

### Doxygen Configuration
```doxyfile
# Doxyfile configuration for STM32G4 project
PROJECT_NAME           = "Digital Power Control"
PROJECT_NUMBER         = 1.0.0
PROJECT_BRIEF          = "STM32G4 Digital Power Control Firmware"

INPUT                  = src include
RECURSIVE              = YES
EXCLUDE                = tests mocks

GENERATE_HTML          = YES
GENERATE_LATEX         = NO
GENERATE_XML           = YES
GENERATE_TAGFILE       = doxygen.tag

EXTRACT_ALL            = YES
EXTRACT_PRIVATE        = YES
EXTRACT_STATIC         = YES

WARN_IF_UNDOCUMENTED   = YES
WARN_IF_DOC_ERROR      = YES

# STM32G4 specific settings
PREDEFINED             = STM32G474xx USE_HAL_DRIVER
MACRO_EXPANSION        = YES
EXPAND_ONLY_PREDEF     = YES
```

### Sphinx Documentation
- **Integration**: Combine Doxygen XML with Sphinx for comprehensive docs
- **Diagrams**: Generate architecture diagrams with PlantUML
- **PDF Output**: Generate printable documentation

## 5. Configuration Management

### Parameter Management System
- **Runtime Configuration**: Load parameters from non-volatile memory
- **Parameter Validation**: Check parameter ranges and relationships
- **Version Compatibility**: Handle parameter set evolution

```c
// Parameter management system
typedef struct {
    uint32_t version;
    uint32_t checksum;
    system_config_t config;
} parameter_block_t;

#define PARAMETER_BLOCK_ADDRESS    0x0801F000  // Flash sector

bool load_parameters(system_config_t *config) {
    parameter_block_t *block = (parameter_block_t *)PARAMETER_BLOCK_ADDRESS;

    // Version check
    if (block->version != CURRENT_PARAMETER_VERSION) {
        return false;
    }

    // Checksum validation
    if (calculate_checksum(&block->config) != block->checksum) {
        return false;
    }

    *config = block->config;
    return true;
}

bool save_parameters(const system_config_t *config) {
    parameter_block_t block;
    block.version = CURRENT_PARAMETER_VERSION;
    block.config = *config;
    block.checksum = calculate_checksum(config);

    return flash_write_block(PARAMETER_BLOCK_ADDRESS, &block, sizeof(block));
}
```

### Board Variant Management
- **Conditional Compilation**: Different features for different boards
- **Pin Mapping**: Board-specific pin configurations
- **Resource Allocation**: Different peripherals for different variants

```c
// Board variant configuration
#if defined(BOARD_VARIANT_A)
#define ADC_CHANNEL_CURRENT    ADC_CHANNEL_1
#define PWM_TIMER             TIM1
#define PWM_CHANNEL           TIM_CHANNEL_1

#elif defined(BOARD_VARIANT_B)
#define ADC_CHANNEL_CURRENT    ADC_CHANNEL_2
#define PWM_TIMER             TIM8
#define PWM_CHANNEL           TIM_CHANNEL_2

#else
#error "Unknown board variant"
#endif
```

## 6. Debugging and Monitoring

### Real-time Debugging Framework
- **Debug Console**: UART-based command interface
- **Variable Monitoring**: Runtime inspection of control variables
- **Performance Profiling**: ISR timing and CPU usage statistics

```c
// Debug console commands
typedef void (*debug_command_t)(char *args);

typedef struct {
    const char *name;
    debug_command_t handler;
    const char *help;
} debug_command_entry_t;

const debug_command_entry_t debug_commands[] = {
    {"status", cmd_status, "Show system status"},
    {"params", cmd_show_params, "Display control parameters"},
    {"set", cmd_set_param, "Set control parameter"},
    {"reset", cmd_reset_controller, "Reset control loops"},
    {"perf", cmd_show_performance, "Show performance statistics"},
};

void debug_console_process(char *line) {
    char *command = strtok(line, " ");
    char *args = strtok(NULL, "");

    for (size_t i = 0; i < ARRAY_SIZE(debug_commands); i++) {
        if (strcmp(command, debug_commands[i].name) == 0) {
            debug_commands[i].handler(args);
            return;
        }
    }

    debug_printf("Unknown command: %s\r\n", command);
}
```

### Data Logging System
- **Circular Buffer**: Continuous data capture without stopping execution
- **Trigger Conditions**: Log data around fault events
- **Compression**: Reduce data volume for long-term logging

## 7. Release Management

### Firmware Update Mechanism
- **In-field Updates**: Support for firmware updates without JTAG
- **Rollback Protection**: Ability to revert to previous version
- **Version Tracking**: Embedded version information and build metadata

```c
// Firmware version information
typedef struct {
    uint32_t magic;           // Identification
    uint16_t major;
    uint16_t minor;
    uint16_t patch;
    uint32_t build_number;
    char build_date[16];
    char build_time[16];
    uint32_t checksum;
} firmware_info_t;

const firmware_info_t firmware_info __attribute__((section(".firmware_info"))) = {
    .magic = FIRMWARE_MAGIC,
    .major = 1,
    .minor = 0,
    .patch = 0,
    .build_number = BUILD_NUMBER,
    .build_date = __DATE__,
    .build_time = __TIME__,
    .checksum = 0  // Calculated at build time
};
```

### Binary Distribution
- **Secure Distribution**: Cryptographic signatures for firmware images
- **Compatibility Matrix**: Document hardware and software compatibility
- **Migration Guide**: Instructions for updating from older versions

## 8. Team Collaboration

### Code Review Process
- **Pull Request Template**: Standardized format for code changes
- **Review Checklist**: Ensure all quality criteria are met
- **Automated Reviews**: Use tools for basic checks

### Knowledge Sharing
- **Documentation Wiki**: Centralized knowledge base
- **Code Examples**: Reusable patterns and templates
- **Training Materials**: Onboarding documentation for new team members

### Issue Tracking Integration
- **Commit Message Standards**: Link commits to issues
- **Automated Changelog**: Generate release notes from commit history
- **Metrics Tracking**: Monitor development velocity and quality trends
