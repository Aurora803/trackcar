# Track Car Hardware Pin Map Draft

Source project: `D:\Development\Projects\stm32\trackcar`

Confirmed by user:

- MCU: STM32F103C8T6
- Motor supply: 12 V lithium battery
- Motor driver: TB6612FNGVM, VM from 12 V battery
- Buck regulator: LM2596 from 12 V to 5 V
- Remaining 5 V / 3.3 V design can be drafted and revised later

Design assumptions for this draft:

- MCU package: STM32F103C8T6, LQFP48.
- MCU logic rail: 3V3 derived from 5V through a 3.3 V LDO.
- TB6612 logic VCC: 3V3, so MCU GPIO can drive it directly.
- Track sensor and encoder interfaces are connector-based because exact modules are not confirmed.
- Track sensor outputs are treated as active-low digital outputs because `LINE_LEVEL` is 0 in code.

## MCU Pin Usage

| MCU pin | Net name | Code name | Direction | Mode | Peripheral | Hardware module | Notes |
|---|---|---|---|---|---|---|---|
| PA0 | LEFT_ENC_A | motor 0 encoder A | Input | Pull-up | TIM2_CH1 | Left encoder | Quadrature encoder input |
| PA1 | LEFT_ENC_B | motor 0 encoder B | Input | Pull-up | TIM2_CH2 | Left encoder | Quadrature encoder input |
| PA2 | USART2_TX | USART2 TX | Output | AF push-pull | USART2_TX | Debug UART | 9600 8N1 |
| PA3 | USART2_RX | USART2 RX | Input | Floating | USART2_RX | Debug UART | 9600 8N1 |
| PA4 | TRACK_X8 | TRACK_X8 | Input | Pull-up | GPIO | Track sensor | Rightmost sensor |
| PA5 | TRACK_X7 | TRACK_X7 | Input | Pull-up | GPIO | Track sensor | Digital input |
| PA6 | TRACK_X6 | TRACK_X6 | Input | Pull-up | GPIO | Track sensor | Digital input |
| PA7 | TRACK_X5 | TRACK_X5 | Input | Pull-up | GPIO | Track sensor | Digital input |
| PA8 | RIGHT_PWM | PWM channel 1 | Output | AF push-pull | TIM1_CH1 | Right motor PWM | 1 kHz, range 0..1000 |
| PA9 | LEFT_PWM | PWM channel 2 | Output | AF push-pull | TIM1_CH2 | Left motor PWM | 1 kHz, range 0..1000 |
| PA13 | SWDIO | SWDIO | Bidirectional | Debug | SWD | Programming/debug | Reserve |
| PA14 | SWCLK | SWCLK | Input | Debug | SWD | Programming/debug | Reserve |
| PB0 | TRACK_X4 | TRACK_X4 | Input | Pull-up | GPIO | Track sensor | Digital input |
| PB1 | TRACK_X3 | TRACK_X3 | Input | Pull-up | GPIO | Track sensor | Digital input |
| PB6 | RIGHT_ENC_A | motor 1 encoder A | Input | Pull-up | TIM4_CH1 | Right encoder | Software reverses right delta |
| PB7 | RIGHT_ENC_B | motor 1 encoder B | Input | Pull-up | TIM4_CH2 | Right encoder | Software reverses right delta |
| PB10 | TRACK_X2 | TRACK_X2 | Input | Pull-up | GPIO | Track sensor | Digital input |
| PB11 | TRACK_X1 | TRACK_X1 | Input | Pull-up | GPIO | Track sensor | Leftmost sensor |
| PB12 | AIN1 | AIN1_PIN | Output | Push-pull | GPIO | TB6612 AIN1 | Left motor direction |
| PB13 | AIN2 | AIN2_PIN | Output | Push-pull | GPIO | TB6612 AIN2 | Left motor direction |
| PB14 | BIN1 | BIN1_PIN | Output | Push-pull | GPIO | TB6612 BIN1 | Right motor direction |
| PB15 | BIN2 | BIN2_PIN | Output | Push-pull | GPIO | TB6612 BIN2 | Right motor direction |
| OSC_IN | HSE_IN | HSE | Input | Clock | RCC | 8 MHz crystal | Required by current clock code |
| OSC_OUT | HSE_OUT | HSE | Output | Clock | RCC | 8 MHz crystal | Required by current clock code |
| NRST | NRST | reset | Input | Reset | Reset | Reset circuit/SWD | Pull-up and reset key |
| BOOT0 | BOOT0 | boot mode | Input | Boot | Boot strap | Boot config | Pull down by default |

## Peripheral Usage

| Peripheral | Purpose | Pins | Configuration from code |
|---|---|---|---|
| TIM1_CH1 | Right motor PWM | PA8 | PWM1, 1 kHz, ARR=999, PSC=71 |
| TIM1_CH2 | Left motor PWM | PA9 | PWM1, 1 kHz, ARR=999, PSC=71 |
| TIM2 | Left wheel quadrature encoder | PA0, PA1 | Encoder mode TI12 |
| TIM3 | Reserved | none | 当前代码未占用。后续云台可评估 TIM3 部分重映射到 PB4/PB5，但不能占用已接循迹的 PB0/PB1 |
| TIM4 | Right wheel quadrature encoder | PB6, PB7 | Encoder mode TI12 |
| USART2 | Debug UART | PA2, PA3 | 9600, 8N1, no flow control |
| SysTick | 1 ms system tick | none | `SysTick_Handler` 调用 `BSP_SysTick_Inc()`，主循环按毫秒轮询调度控制/遥测/LED |
| ADC | Not used | none | No user-level initialization found |
| I2C | Not used | none | No user-level initialization found |
| SPI | Not used | none | No user-level initialization found |
| EXTI | Not used | none | No user-level initialization found |

## Known Pin-Function Conflicts To Avoid

| Pins | Current use | Future conflict |
|---|---|---|
| PA4..PA7 | Track X8..X5 | SPI1 NSS/SCK/MISO/MOSI |
| PB6/PB7 | Right encoder | I2C1 SCL/SDA |
| PB10/PB11 | Track X2/X1 | USART3/I2C2 |
| PB12..PB15 | TB6612 direction pins | SPI2 |
| PA9 | Left PWM | USART1_TX |
| PA2/PA3 | Debug UART / reserved vision UART | OpenMV/RPi UART | 当前只有一组可用 USART2，调试 USB-TTL 与视觉数据不能同时独立连接 |
| PB4/PB5 | Currently free when JTAG disabled | Future TIM3_CH1/CH2 servo PWM | 需使用 TIM3 部分重映射；保持 PB0/PB1 继续给循迹 X4/X3 |
