/*
 * control.c - Control glue
 *
 * 功能：对 PID 控制器进行封装，提供控制模块的初始化和每次循环的控制计算接口。
 */
#include "control.h"
#include "pid_controller.h"

void control_init(const gains_pid_t *gains){
    (void)gains; 
    init_pid_attitude_fixed_height_controller();
}

void control_compute(const actual_state_t *actual, desired_state_t *desired, const gains_pid_t *gains, double dt, motor_power_t *out){
    pid_velocity_fixed_height_controller(*actual, desired, *gains, dt, out);
}
