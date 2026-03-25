# STM32G4 Capability Matrix (Reference)

## Purpose

This reference exists to keep AI-generated guidance from over-assuming that
all `STM32G4xx` parts expose the same peripheral mix, memory size, or package
options.

Use it when a recommendation depends on:
- `CORDIC`
- `HRTIM`
- internal `COMP` or `OPAMP`
- multi-ADC synchronization layouts
- flash / SRAM headroom
- package pin count or external signal routing

## 1. Safe Default Assumption

For generic digital power discussion, it is reasonable to start from a
capable STM32G4 device such as `STM32G474`, then say that part-specific
verification is still required.

Suggested wording:

- Assume an STM32G4 device with FPU and advanced timers
- Prefer `STM32G474`-class capability when discussing digital power features
- Explicitly verify peripheral availability before claiming `CORDIC`,
  `HRTIM`, `COMP`, `OPAMP`, or specific ADC pairings

## 2. What Usually Carries Across The Family

- Cortex-M4 single-precision FPU
- 12-bit ADC architecture concepts
- advanced timer / PWM synchronization concepts
- hard-float embedded C toolchain patterns
- no-heap / no-recursion real-time firmware discipline

These are good family-level assumptions for architectural discussion.

## 3. What Must Be Verified Per Part Number

### Digital Power Peripheral Availability

- `CORDIC`: do not assume it is present without checking the datasheet / reference manual
- `HRTIM`: treat as part-specific capability, not a family-wide guarantee
- `COMP` / `OPAMP`: internal analog feature count and routing options vary
- ADC simultaneous mode options: verify which ADC instances exist and which pairings are supported

### Memory And Packaging

- Flash size and SRAM size
- package pin count and alternate-function availability
- whether the chosen package exposes all PWM, ADC trigger, comparator, and fault pins needed by the topology

### Clock / Timing Envelope

- maximum clock configuration
- timer clock tree details
- whether the intended PWM frequency, dead-time resolution, and ADC timing plan fit the selected device

## 4. Recommendation Policy For AI Output

When giving code or design guidance:

- Say `STM32G4xx (verify exact part number)` unless the user already gave a device
- Say `STM32G474-class assumption` when discussing rich digital power features
- Do not claim a peripheral is present unless the part number is known or the claim is explicitly marked as an assumption
- Do not present memory headroom, ISR budget, or RAM placement gains as guaranteed without measurement

## 5. Verify-Before-Claiming Checklist

Before asserting a hardware-dependent recommendation, verify:

1. Exact MCU part number
2. Package and pinout
3. Presence of `CORDIC`, `HRTIM`, `COMP`, `OPAMP`
4. ADC instance count and simultaneous/injected trigger plan
5. Flash / SRAM budget for buffers, filters, and diagnostics
6. Whether the board-level analog front end matches the intended sample time and protection method

## 6. Example Wording

Good:
- `Assuming an STM32G474-class device with HRTIM and CORDIC, this control path can use hardware trig acceleration; verify the exact part before integrating.`
- `This recommendation is suitable for STM32G4 digital power projects, but ADC pairing and internal analog availability still need part-number confirmation.`

Too strong:
- `All STM32G4 parts can use HRTIM for this topology.`
- `CORDIC is available, so use it in the ISR.`
- `This memory layout will fit on STM32G4.`
