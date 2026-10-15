# Wiring and hardware integration

Signal names below are conceptual; confirm exact MCU pins and timer channels in the board datasheet and CubeMX.

| MCU signal | Connects to | Notes |
|---|---|---|
| Timer STEP output | Driver STEP | Respect minimum pulse width |
| GPIO DIR | Driver DIR | Respect direction setup/hold times |
| GPIO ENABLE | Driver ENABLE | Set safe disabled state at reset; confirm polarity |
| Timer encoder channels A/B | Quadrature encoder A/B | Verify voltage compatibility and pull-ups |
| GPIO/EXTI | Home switch | Debounce before assigning home |
| GPIO/EXTI | Positive and negative limits | Test switch polarity before motor enable |
| UART TX/RX | ST-LINK VCP or USB-UART | Commands and diagnostics |
| GND | Signal ground | Follow driver grounding recommendations |

## Commissioning order

1. Run host tests first.
2. Power the MCU alone and confirm driver-enable stays disabled at reset.
3. Verify encoder direction and counts per revolution by hand.
4. Inspect STEP/DIR timing with a scope or logic analyzer.
5. Validate switch polarity and debouncing with motor power disabled.
6. Begin with low current, low speed, and short travel limits.

Never connect motor windings directly to MCU pins. Use a suitable driver and rated supply. Software STOP is not a physical emergency stop.
