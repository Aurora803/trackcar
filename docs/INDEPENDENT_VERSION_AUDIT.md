# 校赛独立运行版本代码审查与死代码清理报告

## 1. 审查范围与基线

- 清理分支：`competition-independent-clean`；
- 基线标签：`school-competition-line-v1`，提交 `bbaeb48`；
- 清理分支从冻结标签创建，标签本身未修改；
- 原工作分支 `codex/stabilize-p1-telemetry` 的 HEAD 为 `2a110fd`，其相对标签只包含文档更新，STM32 源码与标签一致；
- 用户要求保留的未跟踪文件 `witch -c competition-submission school-competition-line-v1` 未删除、未覆盖、未加入工程；
- 本次未执行 `git commit` 或 `git push`。

## 2. 独立版本架构

独立运行版本采用双控制器解耦架构：STM32 负责底盘循迹和 HC-05 控制；OpenMV 独立完成红色色块识别和 MG996R 水平云台控制，两者不进行数据通信。

```text
STM32F103C8T6                         OpenMV
  8 路循迹传感器                       红色色块识别
  矩形循迹状态机                       水平目标偏差计算
  PID/差速/TB6612                      直接输出 MG996R PWM
  TIM2/TIM4 编码器
  USART2 HC-05 启停与遥测

              无 UART、无数据通信、无任务等待关系
```

仓库当前没有 `OpenMV/` 或 `openmv/` 程序目录。因此本次没有修改或伪造 OpenMV 算法，OpenMV 子系统仍需取得实际脚本后独立验证。

## 3. STM32 实际运行链路

```text
startup -> main
        -> AppRobot_Init
           -> BSP_GPIO_InitAll
           -> BSP_SysTick_Init
           -> BSP_UART_Init (USART2 PA2/PA3, 9600 8N1)
           -> Chassis_Init(TB6612_GetDriver)
              -> TB6612_Init -> TIM1 电机 PWM
              -> BSP_Encoder_Init -> TIM2/TIM4
           -> AppLineFollow_Init(YahboomTracker8IO_GetDriver)
           -> APP_ENABLE_BLUETOOTH_CONTROL=1 时默认 ROBOT_MODE_STOP

while(1) -> AppRobot_Task
         -> 每次循环轮询 HC-05 RX 环形缓冲
         -> 每 10 ms 采样编码器并调度 STOP/LINE_FOLLOW
         -> LINE_FOLLOW -> 8 路采样 -> 状态机/PID -> Chassis -> TB6612
         -> 每 200 ms printf -> USART2 TX 队列 -> USART2_IRQHandler
```

删除前视觉/云台调用只存在于 `#if APP_ENABLE_VISION_TARGET` 条件块；两个开关均为 0，仓库内没有切换到 `ROBOT_MODE_TARGET_TRACK` 的调用者。

## 4. 删除前依赖审查

| 文件/符号 | 删除前作用 | 独立版是否运行 | 删除前引用者 | 结论 | 风险控制 |
|---|---|---:|---|---|---|
| `App/app_vision_target.c/.h` | 解析目标误差并计算 pan/tilt | 否 | `app_robot.c` 条件块、工程清单 | 删除 | 同步删 include、初始化和模式分支 |
| `Components/vision_protocol.c/.h` | 解析 `$T,x,y,valid` | 否 | 仅 `app_vision_target` | 删除 | 同步删 VisionUART 包装 |
| `Components/gimbal_servo.c/.h` | 二维云台角度适配 | 否 | 仅视觉条件块 | 删除 | 同步删 BSP 舵机层和接口 |
| `Components/gimbal_if.h` | 云台驱动抽象 | 否 | 仅两个被删头文件 | 删除 | 已确认无非视觉使用者 |
| `BSP/bsp_servo.c/.h` | TIM1 舵机 PWM 旧占位 | 否 | 仅 `gimbal_servo`、工程清单 | 删除 | 消除误启用时与电机 PWM 冲突的可能 |
| `ROBOT_MODE_TARGET_TRACK` | STM32 视觉专用模式 | 否 | `app_robot.c/.h` | 删除 | 后续移除电机测试 Demo 后，运行模式仅保留 LINE_FOLLOW 和 STOP |
| `BSP_VisionUART_*` | 将视觉接口复用到 USART2 | 否 | 仅 `vision_protocol` | 删除 | 完整保留 Debug UART、RX/TX 队列和 ISR |
| 视觉/云台配置宏 | 关闭上述休眠链路 | 否 | 仅上述模块及冲突保护 | 删除 | 未触及蓝牙、循迹、电机和编码器宏 |

删除前 Keil 会编译 `app_vision_target.c`、`vision_protocol.c`、`gimbal_servo.c` 和 `bsp_servo.c`，并将对象传给链接器；旧 map 明确把这些对象的全部代码/数据段以及 `BSP_VisionUART_*` 全部移除，最终镜像贡献为 0。因此它们属于“参与编译但不进入固件、不参与运行”的死链路。

## 5. 实际删除文件

STM32 视觉/云台链路：

- `App/app_vision_target.c`
- `App/app_vision_target.h`
- `Components/vision_protocol.c`
- `Components/vision_protocol.h`
- `Components/gimbal_servo.c`
- `Components/gimbal_servo.h`
- `Components/gimbal_if.h`
- `BSP/bsp_servo.c`
- `BSP/bsp_servo.h`

额外确认无引用、无构建作用的旧备份：

- `User/main.c.bak-20260627-234449`：旧架构 main 快照，引用已淘汰的 `sys/usart/car_control` 接口；
- `project.uvprojx.bak-20260627-234449`：非生效 Keil 备份，仍包含已删除文件路径。

Git 历史和冻结标签可提供完整回退，不再需要在当前树中保留这两份失效备份。

## 6. 实际修改文件与内容

### 应用和 BSP

- `App/app_config.h`：删除视觉波特率、视觉/云台开关、USART1 视觉预留和 pan/tilt 参数；
- `App/app_robot.c`：删除视觉 include、UART 共用保护、视觉初始化和 `TARGET_TRACK` 分支；
- `App/app_robot.h`：删除 `ROBOT_MODE_TARGET_TRACK`；后续已一并移除 `MOTOR_TEST`，当前仅保留 `LINE_FOLLOW=0`、`STOP=1`；
- `BSP/bsp_uart.c/.h`：只删除 `BSP_VisionUART_*` 包装和视觉专用冲突保护；
- `Components/chassis.c`、`common_types.h`、`pid.c`：只清理已失效的视觉/云台注释，没有功能修改。

### 工程配置

- `project.uvprojx`：移除 9 个被删模块条目；
- `project.uvoptx`：移除相同 9 个文件的 UI/选项记录；
- `.eide/eide.yml`：移除相同 9 个路径；
- 两份 Keil XML 与 EIDE YAML 均通过语法解析；
- 正式 Keil 工程剩余 81 个文件路径，缺失路径为 0；
- 编译输入由 42 个降为 38 个，即移除 4 个死 `.c` 输入。

### Keil 生成记录

- `Objects/project.lnp`：重建后不再把 4 个已删除对象传给链接器；
- `Objects/project.build_log.htm`：刷新为本次 0 Error/0 Warning 构建记录；
- `Objects/project.htm`：刷新为本次镜像组成和大小报告。

### 文档

- `README.md`、`TODO.md`、`pinmap.md`；
- `docs/PINMAP.md`、`PLAN.md`、`PORTING.md`、`RECTANGLE_TRACK.md`、`TUNING.md`、`VALIDATION.md`；
- `docs/OPENMV_RPI_PROTOCOL.md` 保留为通信增强版历史边界参考，并明确不属于独立版本；
- `docs/SCHOOL_COMPETITION_TEST_REPORT.md` 恢复连续 5 圈实车记录并改为独立架构表述；
- 本报告 `docs/INDEPENDENT_VERSION_AUDIT.md`。

## 7. 保留文件及理由

| 文件/模块 | 保留理由 |
|---|---|
| `App/app_line_follow.c/.h` | 已验证矩形循迹状态机，禁止功能修改 |
| `Components/pid.c/.h` | 循迹实际使用 |
| `Components/common_types.h` | PID、循迹、PWM、电机和底盘仍使用 clamp |
| `Components/chassis.c/.h` | 底盘调度、限幅、编码器状态核心 |
| `Components/tb6612_motor.c/.h` | 电机方向与 TB6612 控制核心 |
| `Components/yahboom_tracker8_io.c/.h` | 8 路循迹传感器核心 |
| `BSP/bsp_uart.c/.h` | USART2、HC-05、遥测、环形缓冲和 printf 核心 |
| `BSP_UART1_*` 兼容接口 | 虽无仓库内调用，但属于通用调试兼容 API，不与视觉链路绑定 |
| `Tools/Makefile.template` | 明确标注为迁移模板，不是当前生效构建脚本 |
| `docs/OPENMV_RPI_PROTOCOL.md` | 仅用于说明独立版与通信增强版边界 |

## 8. 对稳定功能的影响

### 循迹

- `App/app_line_follow.c` 无差异；
- PID/纯 P 参数、角点判断、CORNER/BLIND/RECOVER 逻辑均未修改；
- `APP_CONTROL_PERIOD_MS=10U`；
- `LINE_CORNER_ENCODER_TARGET=495`；
- `LINE_RECOVER_CORRECTION_LIMIT=40`；
- `LINE_RECOVER_PWM=200`；
- `LINE_RECOVER_CENTER_CONFIRM_MS=250U`。

### HC-05

- `APP_ENABLE_BLUETOOTH_CONTROL=1`；
- 上电默认 `ROBOT_MODE_STOP`；
- `1`、`START`、`GO` 启动；
- `0`、`STOP` 立即停车；
- 命令解析、ACK 和错误返回均保留。

### USART2 与 printf

- USART2 仍为 PA2/PA3、9600 8N1；
- RX 环形缓冲、TX 队列、TX 丢弃计数均保留；
- `USART2_IRQHandler -> BSP_UART2_IRQHandler` 保留；
- `_write` 和 `fputc` 重定向保留；
- 删除的只是没有运行调用者的 VisionUART 别名。

### 电机、传感器与编码器

- TIM1 PA8/PA9 电机 PWM、1 kHz 配置未修改；
- PB12~PB15 TB6612 方向引脚和真值逻辑未修改；
- 8 路循迹输入 PB11/PB10/PB1/PB0/PA7/PA6/PA5/PA4 未修改；
- TIM2 PA0/PA1、TIM4 PB6/PB7 编码器配置未修改。

## 9. Keil Clean/Rebuild 结果

正式工程：`project.uvprojx`，目标 `Target_1`，STM32F103C8，Keil µVision 5.43.1，Arm Compiler 6.24。

执行结果：

```text
Clean started: Project: 'project'
Clean done

Rebuild target 'Target_1'
Program Size: Code=14368 RO-data=644 RW-data=12 ZI-data=1676
".\Objects\project.axf" - 0 Error(s), 0 Warning(s).
Build Time Elapsed: 00:00:01
```

- 清理前最新同基线 Keil 记录：Code 14384，0 Error，0 Warning；
- 清理后：Code 14368，0 Error，0 Warning；
- Warning 变化：`0 -> 0`；
- Code 减少 16 字节，RO/RW/ZI 不变；
- 新 `project.axf`、`project.hex`、`project.map` 生成时间为 2026-07-13 00:37:10；
- `project.lnp`、新 map、重建日志和三份生效工程配置均不再出现被删模块、符号或路径；
- Keil Clean 不会识别已从工程移除的旧对象，因此额外删除了 8 个 2026-07-11 遗留的 `.o/.d` 生成物；复核残留为 0；
- 生效 Keil 工程失效文件路径为 0。
- Keil 自动生成日志中的 Program Size 行尾空格已做纯格式规范化；最终全工作树 `git diff --check` 为 0。

构建日志位于忽略的构建目录：`Objects/independent_clean.log`、`Objects/independent_rebuild.log`；链接 map 为 `Listings/project.map`。

## 10. 尚未进行的实车测试

本次完成了静态审查和实际 Keil 构建，但没有连接实车、HC-05 或 OpenMV。仍需人工验证：

1. 上电后默认停车；
2. HC-05 发送 `1` 后开始循迹；
3. HC-05 发送 `0` 后立即停车；
4. `START/GO/STOP` 文本命令；
5. USART2 遥测和 ACK；
6. 8 路循迹数据；
7. 左右电机方向；
8. 两路编码器数据；
9. 完整矩形循迹和连续多圈恢复；
10. 实车使用的角点目标仍为 495；
11. OpenMV 可独立上电运行；
12. MG996R 仍由 OpenMV 直接控制；
13. 实物中不存在 STM32/OpenMV UART 依赖；
14. OpenMV 故障不会阻塞 STM32 循迹；
15. STM32 重启不会影响 OpenMV 程序逻辑。

## 11. 独立版与通信增强版边界

本分支只维护双控制器独立运行结构。任何 STM32/OpenMV 数据通信、目标坐标协议、STM32 云台角度计算或 STM32 舵机 PWM，都属于未来单独命名的通信增强分支，不得直接恢复到本分支。

`docs/OPENMV_RPI_PROTOCOL.md` 只保留历史协议需求，不能作为当前接线、构建或验收依据。

## 12. 回退方法

当前尚未提交。若要完全撤销本清理分支的跟踪文件修改，可在确认仍位于 `competition-independent-clean` 后执行：

```powershell
git restore --source=school-competition-line-v1 --worktree -- .
Remove-Item -LiteralPath '.\docs\INDEPENDENT_VERSION_AUDIT.md'
Remove-Item -LiteralPath '.\docs\SCHOOL_COMPETITION_TEST_REPORT.md'
```

上述命令不会删除用户要求保留的未跟踪文件。确认工作区恢复后，如需删除清理分支，应先切回原分支，再手动删除 `competition-independent-clean`；本任务不自动执行该操作。
