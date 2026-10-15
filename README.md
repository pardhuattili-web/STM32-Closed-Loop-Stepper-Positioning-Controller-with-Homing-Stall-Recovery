# STM32 Closed-Loop Stepper Positioning Controller

A portfolio project for precision motion control on STM32. This repository provides a **portable Embedded C control core**, deterministic host simulator, automated unit tests, and a documented path toward STM32 timer, quadrature-encoder, and limit-switch integration.

> **Status:** Host-side control logic and simulator are the first deliverable. The host build does not validate STM32 HAL integration, timing jitter, or physical motor behaviour. Mark hardware support as validated only after building and testing on the selected board.

## What it demonstrates

- Position commands and a bounded motion state machine
- Integer-based speed/stopping-distance helper
- Homing and soft-limit transitions
- Encoder following-error and latched-fault handling
- Separation of portable logic from hardware-specific code
- CMake host tests and GitHub Actions CI

## Build and run

Requirements: CMake 3.16+ and a C11 compiler.

```bash
cmake -S . -B build -DCMAKE_BUILD_TYPE=Release
cmake --build build --parallel
ctest --test-dir build --output-on-failure
./build/motion_demo
```

The demo exercises state transitions using simulated feedback. It is not a substitute for motor/encoder tests.

## Design notes

- Positions are expressed in **encoder counts**; one encoder count is not necessarily one motor step.
- A hardware adapter must convert profile output to timer-generated step pulses and sample encoder position safely.
- The speed helper is a conservative stopping-distance calculation, not yet a full timer-driven trapezoidal trajectory generator.
- Homing must be implemented in the platform layer: seek slowly, debounce the switch, back off, re-approach, then assign the reference coordinate.
- Following-error thresholds depend on encoder resolution, mechanics, load, and speed.

## Suggested hardware

- STM32 Nucleo-F446RE or compatible STM32 board
- Properly configured step/direction driver (for example, DRV8825 or TMC-series module)
- Stepper motor matched to the driver and power supply
- Quadrature encoder measuring the motor shaft or load
- Home and limit switches
- Regulated supply, suitable grounding, and a physical emergency-stop/driver-enable interlock

**Safety:** Never drive a motor directly from MCU GPIO. Observe the driver voltage/current ratings. The firmware STOP command is not a replacement for a physical emergency stop.

## STM32 integration plan

1. Configure one timer output for STEP pulse generation and GPIO for DIR/ENABLE.
2. Configure another timer in encoder mode for quadrature feedback.
3. Add debounced home/limit inputs and safe startup defaults.
4. Write a thin STM32 HAL adapter around the portable controller API.
5. Add UART commands such as `MOVE`, `HOME`, `STOP`, and `STATUS`.
6. Measure real following error, homing repeatability, pulse rate, and stop latency.

Pin assignments are intentionally not fixed: timer channels and pins depend on the board and CubeMX configuration. See [docs/wiring.md](docs/wiring.md).

## Validation plan

Host tests cover portable state logic and the profile helper. Before presenting hardware results, test homing repeatability, moves in both directions, encoder disconnect, injected stall/following error, limit-switch activation, safe reset behaviour, and stop latency. Record board, driver, encoder resolution, load, supply, and measurement method. See [docs/validation-plan.md](docs/validation-plan.md).

## Roadmap

- [x] Portable control core and host build
- [x] Deterministic virtual plant and unit tests
- [x] CI plus architecture/wiring/validation documentation
- [ ] STM32 HAL timer and encoder adapters
- [ ] UART command parser and diagnostics
- [ ] Physical homing and stall/following-error tests
- [ ] Publish measured results and demo video

## License

MIT. See [LICENSE](LICENSE).
