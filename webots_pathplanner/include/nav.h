/*
 * nav.h - Navigation interface
 *
 * 声明导航相关的接口：启动导航、更新导航状态与获取当前目标点。
 */
#ifndef NAV_H
#define NAV_H

#include <webots/gps.h>
#include <webots/inertial_unit.h>
#include "pid_controller.h"

void nav_start(WbDeviceTag gps, WbDeviceTag imu, double goal[4], int segments);
void nav_update(double x_global, double y_global, double altitude, desired_state_t *desired_state, double dt);

// 返回当前目标点（若没有目标返回 0 并不修改输出）
int nav_get_current_target(double *tx, double *ty, double *tz);

#endif // NAV_H
