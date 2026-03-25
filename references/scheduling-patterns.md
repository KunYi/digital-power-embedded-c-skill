# Scheduling Patterns (Reference)

## 1. Layered control tasks
- **Fast Loop (20kHz~40kHz)**: Current control PI, dq transforms, PWM updates - executed in high-priority ISR
- **Slow Loop (5kHz~10kHz)**: Voltage control PI, PLL synchronization, harmonic compensation - executed in timer interrupt
- **Background (1ms)**: System monitoring, fault assessment, communication, user interface - executed in main loop
- **Physical Meaning**: Multi-rate control ensures fast current response while maintaining stable voltage regulation

## 2. ISR rules
- **Execution Constraints**: Keep branches minimal and predictable cost to meet hard real-time deadlines
- **Resource Limits**: Avoid dynamic memory allocation, complex math libraries, or variable-length loops
- **Performance Budget**: Set maximum cycle count (e.g., <25% of PWM period) and verify with hardware counters

## 3. PWM + ADC sync
- **Hardware Synchronization**: TIM1/TIM8 center-aligned PWM update triggers ADC sampling at optimal points
- **DMA Optimization**: Use DMA to automatically transfer ADC results to memory buffers, minimizing CPU load
- **Resource Protection**: Ensure atomic access to shared buffers between ISR and background tasks

## 4. Cross-circle communication
- **Lock-Free Design**: Use volatile flag variables for inter-task communication to avoid blocking
- **Data Flow**: ISR writes status flags, background task reads and clears them atomically
- **Dependency Management**: Avoid circular dependencies that could cause deadlocks or priority inversions

## 5. STM32 guidance
- In STM32CubeIDE, use HAL_TIM_PWM_Start and HAL_ADC_Start_DMA in init phase
- In control ISR, only do time-critical sampling, control, protection, and flag updates; do not add unrelated tick maintenance to the fast path.
