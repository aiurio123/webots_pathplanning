/*
 * control.h - Control module interface
 *
 * 声明控制模块的初始化和控制计算函数（封装 PID 控制逻辑）。
 */
#ifndef CONTROL_H
#define CONTROL_H

#include "pid_controller.h"

void control_init(const gains_pid_t *gains);
void control_compute(const actual_state_t *actual, desired_state_t *desired, const gains_pid_t *gains, double dt, motor_power_t *out);

#endif // CONTROL_H
