# Architecture

The code has three layers:

1. **Portable controller** (`src/motion_controller.c`): tracks homing, move acceptance, soft limits, encoder following error, and latched faults. It has no STM32 HAL dependency.
2. **Speed helper** (`src/motion_profile.c`): calculates a stopping-distance speed hint using integer arithmetic. It is deliberately not described as a complete timer-driven trapezoidal trajectory generator.
3. **Plant/platform layer** (`simulator/` and future STM32 adapter): host simulation is available; real hardware must read encoder counts and issue STEP pulses using timer/DMA or another validated hardware-timed method.

## State model

- `UNHOMED`: position commands are rejected until a reference is established.
- `HOMING`: platform logic seeks a configured switch; after a debounced hit, call `motion_controller_home_switch()`.
- `IDLE`: no move is active.
- `MOVING`: the target differs from measured position.
- `FAULT`: fault is latched; clearing it returns to `UNHOMED` and requires re-homing.

## Safety boundaries

The portable core cannot guarantee physical stopping distance or safe mechanics. Hardware integration must use conservative speed/acceleration/travel limits, debounce switches, default the driver to disabled during reset, and validate pulse timing. Do not implement high-rate pulses using blocking delays.
