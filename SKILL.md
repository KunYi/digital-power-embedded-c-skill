---
name: digital-power-embedded-c
description: Embedded C development, code review, control algorithm implementation, state machine and protection design specification for STM32G4xx digital power projects. Use when Codex needs to write or review C code for digital power systems such as PFC, Vienna, buck, boost, LLC, PSFB, grid-tied or off-grid inverter projects, especially on STM32G4xx, and when tasks involve ISR scheduling, ADC/PWM synchronization, PI/PR/PLL/dq algorithms, MISRA-oriented design, engineering integration constraints, or recoverable protection behavior.
---

# Digital Power Embedded C

## Overview

In digital power control scenarios, prioritize output of practical, easy-to-integrate C code and engineering recommendations suitable for real-time interrupt-cycle operation.

First analyze system boundaries, scheduling relationships, protection priorities, and engineering constraints, then provide interface, implementation, and integration recommendations. Do not sacrifice real-time performance, maintainability, or debug feasibility just to "look advanced."

### When to Read References (Task-Based Routing)

When the issue involves specific implementation patterns, read these references as needed; do not load all at once:

- PI/PR fast implementation, extreme hot-path optimization, variable naming and clamping strategy: read `references/pi-code-variants.md`
- Fast-path PI techniques for high-frequency ISRs: read `references/pi-fast-path.md`
- PR, LPF, NOTCH, IIR 2P2Z etc. digital power algorithm original and optimized versions: read `references/filter-and-resonant-variants.md`
- First-order LPF, multi-stage RMS/LPF cascade, coefficient precomputation, extreme LPF: read `references/lpf-and-rms-variants.md`
- NOTCH FILTER, IIR 2P2Z, dual-biquad structure, discrete coefficient implementation: read `references/notch-and-2p2z.md`
- PLL extreme version, lock-state judgement, sample frequency versus bandwidth tradeoff: read `references/pll-extreme-variants.md`
- dq / Clarke / Park transform original and optimized versions: read `references/dq-transform-optimization.md`
- State machine recovery template, recoverable protection substates and fault recovery chain: read `references/state-machine-recovery-templates.md`
- PI controller call position in fast/slow loops, precomputation strategy, execution order: read `references/scheduling-patterns.md`
- Coupling between controller and OVP/OCP/state machine: read `references/protection-integration.md`
- Architecture design patterns, data flow pipelines, configuration-driven design: read `references/architecture-design-patterns.md`
- Performance analysis, cycle budget, memory optimization, hardware acceleration: read `references/performance-analysis-optimization.md`
- Code quality, MISRA compliance, testing frameworks, static analysis: read `references/code-quality-testing.md`
- Engineering practices, DevOps, CI/CD, documentation, release management: read `references/engineering-practices-devops.md`
- ADC calibration, PWM-ADC synchronization, sample time selection, injected/regular group usage, hardware oversampling, analog watchdog: read `references/adc-calibration-sampling.md`

## Reference Documents (Content Summary)

Each reference file contains the following topics. Use this section to determine if a reference is relevant before loading it:

- `references/pi-code-variants.md`
  - Comparison of standard, fast-path, and extreme PI implementations
  - Saturation clamp, anti-windup, and integral back-calculation coefficient handling
  - Floating-point vs CMSIS DSP / CORDIC code reuse strategies for STM32G4

- `references/pi-fast-path.md`
  - Fast-path PI techniques for high-frequency ISRs
  - Cycle optimization, branch elimination, and STM32G4-specific optimizations
  - Anti-windup strategies and integration with control loops

- `references/filter-and-resonant-variants.md`
  - Discrete design and optimization paths for PR/LFP/NOTCH/IIR2P2Z
  - 2P2Z dual biquad implementation and coefficient pre-computation examples
  - Resonator anti-saturation and distortion compensation

- `references/lpf-and-rms-variants.md`
  - First-order LPF, cascaded second-order LPF, and multi-stage RMS sampling filters
  - Sampling frequency/bandwidth tradeoffs, numeric accuracy, and latency analysis
  - Fast coefficient pre-calculation and API conversion strategies

- `references/notch-and-2p2z.md`
  - NOTCH and 2P2Z structures for disturbance rejection
  - Digital coefficient fixed-point strategies and FPU/CORDIC decision points
  - Cascaded biquad vs direct IIR implementations with STM32G4 optimizations

- `references/pll-extreme-variants.md`
  - Comparison of SRF-PLL, DSOGI-PLL, DDSRF, and EPLL
  - Lock-state detection, robustness strategies, and abnormal deviation handling
  - Bandwidth/sample-rate/delay tradeoffs for aggressive tuning with CORDIC trig acceleration

- `references/dq-transform-optimization.md`
  - Clarke/Park/Inverse Park basic and optimized implementations
  - positive/negative sequence definitions, normalization conventions, and angle sync strategies
  - multi-rate angle update and ISR/slower-loop synchronization compensation with CORDIC acceleration

- `references/state-machine-recovery-templates.md`
  - Fault detect/confirm/shutdown/recovery stage template
  - recoverable protection sub-states, auto-recovery chain, soft restart strategy
  - state machine and protection logic decoupling with priority rules

- `references/scheduling-patterns.md`
  - fast-loop/slow-loop/1ms-task layered scheduling recommendations
  - ISR load, branch complexity, time budgeting, and masking strategy
  - timing guidelines for critical paths (ADC sync, PWM update, CAN reporting)

- `references/protection-integration.md`
  - integration framework for OVP/OCP/PG/Fault with main controller
  - protection priority verification, fast shutdown, and anti-chattering strategies
  - post-trip state reset, hardware signal and software lock handling

- `references/architecture-design-patterns.md`
  - Event-driven control loops, data flow pipelines, and configuration-driven design
  - State-based control architecture, memory optimization, and safety patterns
  - STM32G4-specific optimizations including CORDIC integration and DMA-driven data flow

- `references/performance-analysis-optimization.md`
  - Real-time performance metrics, cycle budget analysis, and memory bandwidth optimization
  - Algorithm optimization strategies, compiler techniques, and hardware acceleration utilization
  - Power consumption optimization and debugging/profiling tools

- `references/code-quality-testing.md`
  - MISRA C compliance strategy, static analysis integration, and unit testing frameworks
  - Integration testing strategies, code review checklists, and documentation standards
  - Continuous integration pipeline and version control/release management

- `references/engineering-practices-devops.md`
  - Development environment setup, build system automation, and CI/CD pipeline configuration
  - Documentation generation, configuration management, and debugging/monitoring frameworks
  - Release management, team collaboration practices, and DevOps integration

- `references/adc-calibration-sampling.md`
  - STM32G4 ADC hardware self-calibration, software offset compensation, and OFR register usage
  - PWM–ADC synchronization with TIM1/TIM8 triggers, injected vs regular group selection
  - Dual-ADC simultaneous sampling, DMA double-buffer, hardware oversampling strategies
  - Sample time selection guidelines, ISR read patterns, scaling coefficient precomputation
  - Analog watchdog (AWD) for hardware OVP/OCP detection and common ADC pitfalls

## Highest Priority Preferences

- Prefer `float32_t`
- Prefer `STM32G4xx`
- Use descriptive, engineering-focused comments
- Comments should prioritize variable physical meaning, execution timing, clamp logic, protection priority, scheduling ownership, and engineering risk points
- Prefer fast loop / slow loop / 1ms task layering
- Make protection higher priority than normal control loops
- Output analysis first, then code
- Do not use overly abstract HAL wrappers

## Default Technical Baseline

### Target Platform

Prioritize STM32G4xx MCU series:
- `STM32G4xx` (e.g., STM32G474, STM32G431) with FPU, CORDIC, and advanced timers for digital power control
- Leverage STM32G4 Digital Power architecture: high-resolution timers, CORDIC for trig functions, and optimized PWM/ADC synchronization
- Use CORDIC hardware accelerator for sin/cos operations in PLL and dq transforms instead of software libraries for better performance

### Language And Implementation Baseline

- Use `C99` or `C11` (C11 preferred for better atomic operations if needed)
- Default main compute type `float32_t`
- Default target STM32G4xx FPU (single-precision hardware floating point)
- Can switch to CMSIS DSP / fixed-point / CORDIC per requirement
- Leverage CORDIC hardware for trigonometric operations (sin, cos, atan2) in control algorithms
- No dynamic memory allocation
- No recursion
- No variable length arrays
- Do not compare floats with direct equality
- Do not rely on complex runtime library features

## Core Workflow

### 1. Analyze System Boundaries Before Writing Code

When user requirements are incomplete, first fill in context and clearly state “following default assumptions”:
- MCU model
- compiler environment
- interrupt frequency / call period
- controlled object and topology
- ADC sampling frequency & PWM sync relationship
- IO physical meaning and units
- use float or fixed point
- DSP library allowed or not
- prefer direct register write or DriverLib
- MISRA style required or not
- existing project framework / naming convention

If not provided, use minimal common digital power assumptions but explicitly note them; don’t pretend user gave it.

Default assumption template, explicitly stated in output:
- MCU: `STM32G4xx` (e.g., STM32G474 / STM32G431)
- compiler: `arm-none-eabi-gcc` or `STM32CubeIDE` (C99/C11 compatible)
- fast ISR: `40 kHz`
- slow loop: `10 kHz`
- background task: `1 ms`
- math type: `float32_t`
- PWM update: `TIM1/TIM8 update event at counter zero`
- ADC trigger: `TIM1/TIM8 PWM synchronized to switching period`
- code style: `MISRA-oriented, no dynamic memory, no recursion`

### 2. Identify Task Type Before Choosing Output Format

Based on user request, choose output strategy.

#### Code Generation
If user asks “write code / implement function / complete module”, output:
1. assumptions
2. interface/data structures
3. full implementation
4. invocation, scheduling, integration notes

#### Code Review
If user gives existing code for analysis:
1. logic issues
2. real-time risk
3. maintainability issues
4. MISRA risk points
5. optimized code

#### Control Structure Design
For state machine, protection, scheduling:
1. role partition
2. execution timing
3. transition conditions
4. code framework

#### Algorithm Implementation
For pi/pr/pll/dq/pwm/filter:
- give discrete form and engineering notes
- variable physical meaning
- clamping, anti-windup, init, abnormal input handling
- call period/order
- relation to ADC/PWM/protection

### 3. Preserve Real-Time Behavior And Engineering Executability

- Only keep hard real-time tasks in ISRs
- push slow tasks to lower frequency
- PWM update via shadow register (CTR=ZERO) preferred
- clearly define ADC & PWM trigger relation
- avoid unnecessary branches, long loops, repeated calculations inside ISRs
- be careful with expensive ops (division/sqrt/trig) - use CORDIC for trig when possible
- precompute coefficients outside ISR
- protection priority, state transition priority, PWM shutdown path must be explicit
- do not output isolated code fragments detached from scheduling relation

### 4. Follow MISRA-Oriented Design Principles

Default focus on MISRA C:2012 Required Rules:
- undefined behavior protection
- explicit types
- variable initialization
- return value handling
- array bounds
- null pointer checks
- constant naming
- complete interface prototype
- no recursion
- no dynamic memory

Also explicitly state:
- generated code is “MISRA oriented”
- do not claim “fully MISRA compliant”
- if using GCC attributes/register mapping/ISR attributes/section placement, explicitly call this an engineering exception

### 5. Do Not Overclaim Performance Or Verification Results

Without actual compile output/map/stats/testing data, only “estimate/mock validation” for:
- cycle count
- stack usage
- FPU/CORDIC gain
- RAMFUNC benefit ratio
- THD/PF/margin final values
- full MISRA compliance
- production-ready/automotive-ready

## Algorithm Preferences

### PI / PR Controller

- Prioritize discrete-friendly standard structure
- must contain output limit
- must contain integrator init
- must contain anti-windup
- must contain abnormal input protection
- if back-calculation used, explain coefficient rationale
- if asked “fast/hot-path/extreme/short-cycle PI”, read references files

### dq / Clarke / Park Transform

- specify angle source
- specify positive/negative sequence definition
- specify normalization
- minimize duplicate multiplies and unnecessary temp variables
- use CORDIC for sin/cos computation in high-frequency paths
- if fast ISR and low-speed PLL desync issue exists, warn and give compensation
- if asked “dq optimized/hot-path/low-mul”, read ref file

### PLL

- default to srf-pll
- if asked, can support dsogi-pll/ddsrf/epll
- specify sampling frequency
- specify bandwidth recommendation
- specify lock behavior
- describe impact of distortion/unbalance/bias- leverage CORDIC for trigonometric calculations in angle estimation- if asked “PLL extreme/hot-path/lock-state template”, read ref file

### LPF / NOTCH / IIR 2P2Z / PR

- explain discrete form and coefficient source first, then code
- explain state variable physical meaning and execution sequence
- if asked “optimized/extreme/hot-path”, read ref files
- if requested first-order/RMS cascade/precomputed LPF, read ref file
- if requested NOTCH/IIR2P2Z/biquad/PR resonator implementation, read ref file

### Protection And State Machine

- prefer layered expression: fault detect / confirm / shutoff / recovery
- keep state machine responsibility clear
- ensure protection priority over normal control
- clarify shutoff, relay action, PWM disable, integrator reset order
- if controller output couples with OVP/OCP/state machine, read ref file
- for fault recovery template or RUN-substate auto-recovery, read ref file

## Default Response Style

- first identify if user asked “analysis” or “code”
- no exaggeration, no fake verification
- prioritize truly integrable, debuggable, maintainable implementations
- if user plan has problems, point out directly with reason
- answers must emphasize physical meaning, path, timing, protection priority
- when multiple viable solutions, provide safer engineering one and explain trade-offs
- unclear items must state “assumption/estimate/suggest verify”

## Comment Style

Comments should serve integration; no fluff.
Prioritize:
- variable physical meaning
- execution timing
- clamping logic
- protection priority
- scheduling ownership
- engineering risk points

## Trigger Scope

When request touches:
- STM32G4xx / STM32G474 / STM32G431 etc with CORDIC, FPU, and Digital Power peripherals
- FPU/CORDIC/DSP library/float32_t/fixed-point
- PFC/Vienna/buck/boost/LLC/PSFB/inverter
- digital power control / current loop / voltage loop / dq / pll / pwm / adc sync
- state machine / protection / scheduling / ramfunc / ISR optimization
- MISRA/safety/automotive coding
Then use this skill behavior.

Also triggers on phrases like:
- “PI extreme/hot-path/short-cycle/current loop acceleration/outer loop feedforward”
- “PR optimized/LPF optimized/NOTCH FILTER/2P2Z/biquad/IIR”
- “PLL extreme/SRF-PLL optimized/DSOGI-PLL/dq optimized/Park optimized”
- “fast/slow loop layering/ISR scheduling/precomputed coefficients/controller integrator recovery”
- “OVP recovery/recoverable protection/RUN substate/fault chain/state observation”- "architecture patterns/data flow pipeline/configuration-driven design"
- "performance optimization/cycle budget/memory layout/hardware acceleration"
- "code quality/MISRA compliance/unit testing/static analysis"
- "CI/CD pipeline/documentation generation/release management/DevOps"

## References (Complete File List)

Quick-scan index of all available reference files. Read as needed; do not expand everything:

- `references/pi-code-variants.md`
- `references/pi-fast-path.md`
- `references/filter-and-resonant-variants.md`
- `references/lpf-and-rms-variants.md`
- `references/notch-and-2p2z.md`
- `references/pll-extreme-variants.md`
- `references/dq-transform-optimization.md`
- `references/state-machine-recovery-templates.md`
- `references/scheduling-patterns.md`
- `references/protection-integration.md`
- `references/architecture-design-patterns.md`
- `references/performance-analysis-optimization.md`
- `references/code-quality-testing.md`
- `references/engineering-practices-devops.md`
- `references/adc-calibration-sampling.md`

## Final Goal

- Always aim for "digital power control code that is truly integrable, debuggable, maintainable."
- When info is insufficient, first fill boundary conditions.
- When code risk exists, highlight it.
- When implementation asked, provide complete practical version.
- When real-time is involved, ensure deterministic execution, protection priority, and engineering verifiability.
