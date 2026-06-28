#include "app_robot.h"

int main(void)
{
    AppRobot_Init();

    while (1)
    {
        AppRobot_Task();
    }
}
