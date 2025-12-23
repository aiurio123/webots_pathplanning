/*
 * nav.c - Navigation helper
 *
 * 功能：负责跟踪当前航点、将全局航点转换为期望速度/高度命令并提供状态输出接口。
 */
#include "nav.h"
#include <math.h>
#include <stdio.h>
#include "path_planner.h" // provides waypoints, waypoint_count, init_global_path

static int current_wp = 0;

void nav_start(WbDeviceTag gps, WbDeviceTag imu, double goal[4], int segments){
    // 初始化全局路径（path_planner 提供）
    init_global_path(gps, imu, goal, segments);//你的设计关键入口
    current_wp = 0;
}

void nav_update(double x_global, double y_global, double altitude, desired_state_t *desired_state, double dt){
    if (current_wp >= waypoint_count) current_wp = waypoint_count > 0 ? waypoint_count - 1 : 0;

    if (waypoint_count == 0) return; // no waypoints

    double target_x = waypoints[current_wp][0];
    double target_y = waypoints[current_wp][1];
    double target_z = waypoints[current_wp][2];

    // 位置误差
    double ex = target_x - x_global;
    double ey = target_y - y_global;
    double ez = target_z - altitude;

    double pos_k = 0.6;
    double cmd_vx = pos_k * ex;
    double cmd_vy = pos_k * ey;

    // 速度限幅
    double max_v = 1.0;
    if (cmd_vx > max_v) cmd_vx = max_v;
    if (cmd_vx < -max_v) cmd_vx = -max_v;
    if (cmd_vy > max_v) cmd_vy = max_v;
    if (cmd_vy < -max_v) cmd_vy = -max_v;

    double cmd_alt = target_z;

    // 计算目标航向角
    double dx = target_x - x_global;
    double dy = target_y - y_global;
    double desired_yaw = atan2(dy, dx);


    double yaw_k = 1.2;  // yaw 误差增益

    if (fabs(ex) < 0.10 && fabs(ey) < 0.10 && fabs(ez) < 0.10)
    {
        printf(">>> 到达航点 %d (%.3f, %.3f, %.3f)\n",
            current_wp, target_x, target_y, target_z);

        if (current_wp < waypoint_count - 1)
            current_wp++;
        else
            printf(">>> 所有航点完成。\n");
    }

    desired_state->vx = cmd_vx;
    desired_state->vy = cmd_vy;
    desired_state->altitude = cmd_alt;
    desired_state->yaw_rate = yaw_k * 0.0; 
    
    printf(
        "WP:%d  pos(%.2f, %.2f) → tgt(%.2f, %.2f) | "
        "vx:%.2f vy:%.2f → cmd_vx:%.2f cmd_vy:%.2f | "
        "tgt_z:%.2f\n",
        current_wp,
        x_global, y_global,
        target_x, target_y,
        desired_state->vx, desired_state->vy,
        desired_state->vx, desired_state->vy,
        desired_state->altitude
    );
}

int nav_get_current_target(double *tx, double *ty, double *tz){
    if (waypoint_count == 0) return 0;
    int idx = current_wp >= waypoint_count ? waypoint_count-1 : current_wp;
    if (tx) *tx = waypoints[idx][0];
    if (ty) *ty = waypoints[idx][1];
    if (tz) *tz = waypoints[idx][2];
    return 1;
}
