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
    NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);

    AppRobot_Init();

    while (1)
    {
        AppRobot_Task();
    }
}
