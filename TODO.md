# Hardware TODO / Confirmation List

Independent-version boundary: STM32 handles line following and HC-05 control/telemetry.
OpenMV is a separate controller for red-blob detection and direct MG996R horizontal-servo
control. There is no data link between the controllers. This repository currently contains
no OpenMV program directory, so OpenMV behavior remains an external item to supply and test.

Confirmed in this round:

- MCU is STM32F103C8T6.
- Main battery input is 12 V lithium battery.
- 12 V feeds TB6612FNGVM VM.
- LM2596 generates the 5 V rail from 12 V.
- Draft may derive 3.3 V from 5 V and use reasonable placeholder choices for review.

High-priority confirmations before ordering PCB:

1. Confirm TB6612FNGVM exact form: bare TB6612FNG IC, breakout module, or a specific VM-board footprint.
2. Confirm LM2596 form: module footprint or discrete LM2596S circuit.
3. Confirm 12 V lithium battery range: 2S, 3S, fully charged voltage, connector type, expected peak motor current.
4. Confirm whether TB6612FNGVM can safely handle the selected motor stall current.
5. Confirm motor connector type and wire current rating.
6. Confirm track sensor module power voltage and output voltage.
7. Confirm whether track module outputs are active-low as code currently assumes.
8. Confirm encoder power voltage and output type: push-pull, open-drain, Hall, optical, etc.
9. Confirm right encoder direction reversal in code matches real wiring.
10. Confirm PCB mechanical size, mounting hole positions, and connector directions.
11. Confirm the separate OpenMV and MG996R power, mounting, range, and wiring; do not reserve
    STM32 UART or servo/gimbal headers for them in the independent version.

Assumptions used for the current draft:

- STM32F103C8T6 uses LQFP48.
- MCU rail is 3.3 V.
- TB6612 logic VCC is 3.3 V.
- Track and encoder connectors are powered from 3.3 V unless level shifting is added.
- HSE 8 MHz crystal is included because the firmware configures 72 MHz using HSE.
- TB6612 STBY is pulled up to 3.3 V, not controlled by MCU.

Firmware follow-up items:

1. USART2 `printf` now uses an interrupt-driven TX queue and keeps the Bluetooth link at
   9600 baud with a 200ms telemetry period. Before adding more telemetry, monitor
   `BSP_DebugUART_GetTxDroppedCount()` and keep the queue from overflowing.
2. Bluetooth START/STOP commands cannot detect a wireless disconnect through UART alone.
   Add the module STATE pin or a heartbeat timeout if disconnect-to-stop is required.
3. Obtain and version the actual OpenMV program before competition verification. Do not add
   STM32 vision-protocol, target-tracking, or two-axis gimbal placeholders back to this branch.
