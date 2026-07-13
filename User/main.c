/**
 * @file main.c
 * @brief STM32F103C8T6 循迹小车应用入口。
 * @layer User
 *
 * main.c 只负责 NVIC 分组、应用初始化和主循环调度。
 * 具体循迹、PID、电机、编码器、串口等逻辑由 App/Components/BSP 分层实现。
 */
#include "stm32f10x.h"
#include "app_robot.h"

int main(void)
{
    /* 抢占优先级 2 bit、子优先级 2 bit：SysTick/USART 的具体优先级由各 BSP
     * 初始化设置；控制算法始终留在主循环，避免在中断中执行耗时逻辑。
     */
    NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);

    AppRobot_Init();

    while (1)
    {
        /* AppRobot_Task 内部基于 SysTick 的时间差调度，空转主循环可同时保证
         * 蓝牙 STOP 命令尽快被处理。
         */
        AppRobot_Task();
    }
}
