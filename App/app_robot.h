/**
 * @file app_robot.h
 * @brief 机器人应用层总调度接口。
 * @layer App
 *
 * main.c 只调用本模块完成初始化和循环任务调度。具体循迹算法、
 * 电机驱动、传感器采样均由下层模块完成。
 */
#ifndef APP_ROBOT_H
#define APP_ROBOT_H

#ifdef __cplusplus
extern "C" {
#endif

typedef enum
{
    /* 8 路红外循迹模式，由蓝牙 START/1 命令进入。 */
    ROBOT_MODE_LINE_FOLLOW = 0,
    /* 视觉目标跟踪预留模式，当前默认配置不启用视觉/云台输出。 */
    ROBOT_MODE_TARGET_TRACK,
    /* Left/right wheel open-loop speed test demo. */
    ROBOT_MODE_MOTOR_TEST,
    /* 停车模式，底盘空转停止。 */
    ROBOT_MODE_STOP
} robot_mode_t;

/**
 * @brief 初始化 BSP、底盘、循迹和可选视觉/云台模块。
 */
void AppRobot_Init(void);

/**
 * @brief 主循环周期任务。内部按毫秒 tick 调度控制、遥测和 LED。
 */
void AppRobot_Task(void);

/**
 * @brief 切换机器人运行模式。
 * @note 切到 STOP 时立即停止；从其他模式进入 LINE_FOLLOW 时会复位循迹状态机。
 */
void AppRobot_SetMode(robot_mode_t mode);

/**
 * @brief 获取当前机器人模式。
 */
robot_mode_t AppRobot_GetMode(void);

#ifdef __cplusplus
}
#endif

#endif /* APP_ROBOT_H */
