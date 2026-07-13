# 校赛最终版：STM32F103 循迹小车

本仓库冻结为校赛提交版本。主控为 STM32F103C8T6，完成 8 路循迹、TB6612FNG 双电机驱动、编码器采样，以及通过 HC-05（USART2）进行启动、停止和遥测。

比赛期间请不要修改 `App/app_config.h` 中的循迹、转角和 PWM 参数；需要改动时，应先在副本中完成实车验证。

## 工程结构

```text
User/        程序入口和中断处理
App/         小车初始化、任务调度、循迹状态机和集中参数
BSP/         GPIO、PWM、编码器、串口、SysTick 底层驱动
Components/  底盘、电机、8 路循迹接口和 PID
Library/     STM32F10x 标准外设库
Start/       CMSIS、系统时钟和启动文件
Project/     工程辅助目录
docs/        接线图（仅保留此类文档）
```

接线以 [docs/PINMAP.md](docs/PINMAP.md) 为准。烧录口保留 `PA13/SWDIO`、`PA14/SWCLK`、`NRST` 和 GND。

## 编译

推荐使用 Keil MDK 打开 `project.uvprojx`，选择 `Target_1`，然后执行 **Rebuild**。

当前工程编译配置如下：

- 芯片：STM32F103C8（Cortex-M3），Flash 64 KB、RAM 20 KB。
- 工具链：Arm Compiler 6（ArmClang）；工程使用 C99 和 GNU 扩展，优化等级为 2。
- 预处理宏：`USE_STDPERIPH_DRIVER`；使用标准外设库时还必须选择中等密度器件宏 `STM32F10X_MD`。
- 头文件目录：`Start`、`User`、`Library`、`App`、`BSP`、`Components`。
- 启动文件：`Start/startup_stm32f10x_md.s`。
- 构建产物：`Objects/project.axf` 和 `Objects/project.hex`。

如使用 GCC/EIDE 迁移，采用 `cortex-m3`、Thumb 指令集，并定义 `USE_STDPERIPH_DRIVER` 和 `STM32F10X_MD`；链接脚本使用 `STM32F103C8_FLASH.ld`。`Tools/Makefile.template` 可作为迁移参考，不能直接构建。

## 烧录

1. 将 ST-Link 与开发板连接：`3.3V`、`GND`、`SWDIO(PA13)`、`SWCLK(PA14)`；建议同时连接 `NRST`。
2. 确认 `BOOT0` 接地（从 Flash 启动），并为开发板供电。
3. 在 Keil 中选择 ST-Link 调试/下载器，执行 **Flash → Download**；也可将 `Objects/project.hex` 导入 STM32CubeProgrammer，通过 SWD 写入。
4. 上电后小车默认停车。HC-05 串口为 9600、8N1，发送 `1` 启动，发送 `0` 停止。

## 版本状态

当前分支 `competition-independent-clean` 为校赛最终代码基线。历史标签 `school-competition-line-v1` 对应此前的循迹冻结点；本 README 和接线图是本次提交保留的唯一工程说明资料。
