# Digital Power Embedded C Skill

An AI skill focused on embedded C development for digital power systems, particularly targeting STM32G4xx microcontrollers. This skill provides expert guidance on control algorithms, state machines, protection designs, and engineering best practices for digital power projects including PFC, Vienna rectifiers, buck/boost converters, LLC resonant converters, PSFB, and grid-tied/off-grid inverters. Compatible with various AI coding assistants such as VS Code Copilot, Claude Code, Codex, and others that support skill-based extensions.

## Features

- **Control Algorithm Implementation**: PI, PR, PLL, dq-transform, and various filter implementations (LPF, NOTCH, IIR 2P2Z)
- **Real-time Optimization**: Fast-path techniques, ISR scheduling, ADC/PWM synchronization
- **State Machine Design**: Recoverable protection behaviors, fault recovery chains
- **Architecture Patterns**: Event-driven control loops, signal processing pipelines, configuration-driven design
- **Code Quality**: MISRA compliance, testing frameworks, static analysis
- **Engineering Practices**: DevOps integration, CI/CD pipelines, documentation standards

## Installation

1. Clone this repository to your local machine:
   ```bash
   git clone https://github.com/KunYi/digital-power-embedded-c-skill.git
   ```

2. Copy the `SKILL.md` file to your AI assistant's workspace or project root (e.g., VS Code workspace for Copilot, or appropriate directory for Claude Code/Codex).

3. Ensure the `references/` directory is accessible for detailed implementation guidance.

## Usage

This skill is designed to work with various AI coding assistants that support skill-based extensions. When working on digital power embedded C projects:

1. Mention "digital power" or specific components (e.g., "PI controller for buck converter") in your code comments or queries.
2. The skill will provide context-aware suggestions for:
   - Code implementation patterns
   - Algorithm optimizations
   - Protection and safety mechanisms
   - Integration best practices

## Reference Documents

The skill includes comprehensive reference materials covering:

- `pi-code-variants.md`: PI controller implementations (standard, fast-path, extreme)
- `pi-fast-path.md`: High-frequency ISR optimizations
- `filter-and-resonant-variants.md`: Digital filter designs and optimizations
- `lpf-and-rms-variants.md`: Low-pass and RMS filter implementations
- `notch-and-2p2z.md`: Notch filters and biquad structures
- `pll-extreme-variants.md`: Phase-locked loop implementations
- `dq-transform-optimization.md`: Clarke/Park transforms
- `state-machine-recovery-templates.md`: Fault recovery patterns
- `scheduling-patterns.md`: Control loop scheduling strategies
- `protection-integration.md`: Safety and protection mechanisms
- `architecture-design-patterns.md`: System architecture patterns
- `performance-analysis-optimization.md`: Cycle budget and memory optimization
- `code-quality-testing.md`: Code quality and testing practices
- `engineering-practices-devops.md`: Development and deployment practices

## Contributing

Contributions are welcome! Please feel free to submit issues, feature requests, or pull requests.

## License

This project is licensed under the MIT License - see the [LICENSE](LICENSE) file for details.

## Author

KUNYI CHEN <kunyi.chen@gmail.com>

## Acknowledgments

This skill is inspired by the article "我把数字电源开发经验做成了一个 AI Skill" (I Turned My Digital Power Development Experience into an AI Skill) by 楊帥鍋, published on the WeChat public account "開關電源仿真與實用設計".

Article URL: https://mp.weixin.qq.com/s/Wktyw_3-5TgVvmxBvuFmSw
