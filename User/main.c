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
