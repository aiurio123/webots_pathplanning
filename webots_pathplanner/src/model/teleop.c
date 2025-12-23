/*
 * teleop.c - Teleoperation handler
 *
 * 功能：实现键盘手动控制逻辑并平滑输出速度/高度命令。
 */
#include "teleop.h"
#include <webots/keyboard.h>
#include <stdio.h>

void teleop_handle(desired_state_t *desired_state,
                   double *smoothed_vx, double *smoothed_vy, double *smoothed_alt,
                   double dt){
    int key = wb_keyboard_get_key();
    double forward = 0, sideways = 0, yaw_cmd = 0, dz_cmd = 0;

    while (key > 0)
    {
        if (key == WB_KEYBOARD_UP) forward = 0.5;
        else if (key == WB_KEYBOARD_DOWN) forward = -0.5;
        else if (key == WB_KEYBOARD_LEFT) sideways = 0.5;
        else if (key == WB_KEYBOARD_RIGHT) sideways = -0.5;
        else if (key == 'Q' || key == 'q') yaw_cmd = 1.0;
        else if (key == 'E' || key == 'e') yaw_cmd = -1.0;
        else if (key == 'W' || key == 'w') dz_cmd = 0.15;
        else if (key == 'S' || key == 's') dz_cmd = -0.15;
        key = wb_keyboard_get_key();
    }

    // 高度平滑
    double desired_alt_cmd = *smoothed_alt + dz_cmd * dt;
    const double max_climb_rate = 0.6; // 最大爬升速度  (m/s)
    double max_dz = max_climb_rate * dt;
    double dz_allowed = desired_alt_cmd - *smoothed_alt;
    if (dz_allowed > max_dz) dz_allowed = max_dz;
    if (dz_allowed < -max_dz) dz_allowed = -max_dz;
    *smoothed_alt += dz_allowed;

    // 速度平滑
    double cmd_vx = forward, cmd_vy = sideways;
    const double max_accel = 0.8;      // 最大横向加速度 (m/s^2)
    double dvx = cmd_vx - *smoothed_vx;
    double dvy = cmd_vy - *smoothed_vy;
    double max_dv = max_accel * dt;
    if (dvx > max_dv) dvx = max_dv;
    if (dvx < -max_dv) dvx = -max_dv;
    if (dvy > max_dv) dvy = max_dv;
    if (dvy < -max_dv) dvy = -max_dv;

    *smoothed_vx += dvx;
    *smoothed_vy += dvy;

    desired_state->vx = *smoothed_vx;
    desired_state->vy = *smoothed_vy;
    desired_state->altitude = *smoothed_alt;
    desired_state->yaw_rate = yaw_cmd;
}
