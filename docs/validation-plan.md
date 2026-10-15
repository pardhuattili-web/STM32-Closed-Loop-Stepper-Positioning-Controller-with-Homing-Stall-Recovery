# Validation plan

## Host-only validation

Run the CMake build and unit tests in the README. These validate portable state-machine and profile-helper logic; they do not validate STM32 registers, interrupt timing, or physical movement.

## Record hardware results before claiming completion

| Test | Procedure | Results to record |
|---|---|---|
| Clean build | Build with exact CubeIDE/CubeMX setup | Tool versions, warnings |
| Reset safe state | Reset/power-cycle with driver attached | Enable pin state |
| Encoder direction | Rotate shaft a known amount in both directions | Counts/revolution, sign |
| Homing repeatability | Run at least 20 homing cycles | Reference spread |
| Position accuracy | Move both directions at multiple targets | Target, measured position, error |
| Stall fault | Inhibit virtual/physical movement while commanded to move | Detection time and state |
| Encoder drift | Change feedback while idle beyond the configured tolerance | Fault code and detection behaviour |
| Limit switches | Trigger during a low-speed move | Stop response and false triggers |
| Stop latency | Stop at several configured speeds | Time and travel distance |

Record hardware, load, supply, encoder resolution, pulse rate, and measurement method. Do not infer safety certification or guaranteed worst-case behaviour from a small test set.
