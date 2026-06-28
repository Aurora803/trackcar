#ifndef APP_ROBOT_H
#define APP_ROBOT_H

#ifdef __cplusplus
extern "C" {
#endif

typedef enum
{
    ROBOT_MODE_LINE_FOLLOW = 0,
    ROBOT_MODE_TARGET_TRACK,
    ROBOT_MODE_STOP
} robot_mode_t;

void AppRobot_Init(void);
void AppRobot_Task(void);
void AppRobot_SetMode(robot_mode_t mode);
robot_mode_t AppRobot_GetMode(void);

#ifdef __cplusplus
}
#endif

#endif /* APP_ROBOT_H */
