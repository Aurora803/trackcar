# Keil / EIDE 导入说明

## Keil MDK

1. 新建 STM32F103C8T6 工程；
2. 添加本工程以下目录中的 `.c` 文件：
   - `User/*.c`
   - `App/*.c`
   - `BSP/*.c`
   - `Components/*.c`
3. 添加 ST 标准库源码，至少包括：
   - `stm32f10x_gpio.c`
   - `stm32f10x_rcc.c`
   - `stm32f10x_tim.c`
   - `stm32f10x_usart.c`
   - `stm32f10x_misc.c`
4. Include 路径添加：
   - `User`
   - `App`
   - `BSP`
   - `Components`
   - `CMSIS/...`
   - `STM32F10x_StdPeriph_Driver/inc`
5. Define 添加：
   - `USE_STDPERIPH_DRIVER`
   - `STM32F10X_MD`
6. Startup 选择 `startup_stm32f10x_md.s`。

## EIDE / VSCode

1. 创建空 STM32F103C8T6 标准库工程；
2. 把 `User/App/BSP/Components` 加入源码目录；
3. 确保工具链为 ARM GCC 或 ARMCC；
4. Include/Define 同 Keil；
5. 如果使用 ARM GCC，确认链接脚本与启动文件适配 STM32F103C8T6：Flash 64K，RAM 20K。
