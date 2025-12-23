/*
 * teleop.h - Teleoperation interface
 *
 * 声明键盘控制相关函数与数据结构接口。
 */
#ifndef TELEOP_H
#define TELEOP_H

#include "pid_controller.h"

void teleop_handle(desired_state_t *desired_state,
                   double *smoothed_vx, double *smoothed_vy, double *smoothed_alt,
                   double dt);

#endif // TELEOP_H
