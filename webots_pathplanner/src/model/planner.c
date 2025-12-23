/*
 * planner.c - Top-level planner wrapper
 *
 * 功能：封装高层规划（导航/航向计算），并提供 planner_start/planner_step 接口。
 */
#include "planner.h"
#include "nav.h"
#include <math.h>
#include <stdio.h>

static int planner_inited = 0;

void planner_start(WbDeviceTag gps, WbDeviceTag imu, double goal[4], int segments){
    if (!planner_inited){
        nav_start(gps, imu, goal, segments);
        planner_inited = 1;
    }
}

void planner_step(double x_global, double y_global, double altitude, double current_yaw, desired_state_t *desired_state, double dt){
    if (!planner_inited) return;

    // 更新基础导航速度/高度目标
    nav_update(x_global, y_global, altitude, desired_state, dt);

    // 计算航向误差并写入 yaw_rate
    double tx, ty, tz;
    if (nav_get_current_target(&tx, &ty, &tz)){
        double desired_yaw = atan2(ty - y_global, tx - x_global);
        double yaw_err = desired_yaw - current_yaw;
        while (yaw_err > M_PI) yaw_err -= 2*M_PI;
        while (yaw_err < -M_PI) yaw_err += 2*M_PI;
        double yaw_k = 1.2;
        desired_state->yaw_rate = yaw_k * yaw_err;
    }
}
