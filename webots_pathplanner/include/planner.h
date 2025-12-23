/*
 * planner.h - Planner interface
 *
 * 声明 planner 的启动与运行接口（planner_start/planner_step）。
 */
#ifndef PLANNER_H
#define PLANNER_H

#include <webots/gps.h>
#include <webots/inertial_unit.h>
#include "pid_controller.h"

// 初始化 planner，传入设备和目标
void planner_start(WbDeviceTag gps, WbDeviceTag imu, double goal[4], int segments);

// 每帧调用，传入位置与航向，更新 desired_state（vx, vy, altitude, yaw_rate）
void planner_step(double x_global, double y_global, double altitude, double current_yaw, desired_state_t *desired_state, double dt);

#endif // PLANNER_H
