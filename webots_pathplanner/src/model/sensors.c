/*
 * sensors.c - Sensor processing
 *
 * 功能：读取并封装传感器数据（GPS/IMU/gyro），并提供速度/位置/航向信息给上层模块。
 */
#include "sensors.h"
#include <webots/inertial_unit.h>
#include <webots/gps.h>
#include <webots/gyro.h>
#include <webots/robot.h>
#include <math.h>

void sensors_init(Sensors *s, Devices *dev){
    s->imu = dev->imu;
    s->gps = dev->gps;
    s->gyro = dev->gyro;
    s->past_x = 0.0;
    s->past_y = 0.0;
    s->past_time = wb_robot_get_time();
}

void sensors_update(Sensors *s, actual_state_t *actual_state, double *x_global, double *y_global, double *dt){
    const double *imu_data = wb_inertial_unit_get_roll_pitch_yaw(s->imu);
    actual_state->roll = imu_data ? imu_data[0] : 0.0;
    actual_state->pitch = imu_data ? imu_data[1] : 0.0;
    double actualYaw = imu_data ? imu_data[2] : 0.0;
    s->last_yaw = actualYaw;

    const double *gyro_vals = wb_gyro_get_values(s->gyro);
    actual_state->yaw_rate = gyro_vals ? gyro_vals[2] : 0.0;

    const double *gps_vals = wb_gps_get_values(s->gps);
    double x = gps_vals ? gps_vals[0] : 0.0;
    double y = gps_vals ? gps_vals[1] : 0.0;
    actual_state->altitude = gps_vals ? gps_vals[2] : 0.0;

    double current_time = wb_robot_get_time();
    double local_dt = current_time - s->past_time;
    if (local_dt <= 0) local_dt = *dt;

    double vx_global = (x - s->past_x) / local_dt;
    double vy_global = (y - s->past_y) / local_dt;

    double cosyaw = cos(actualYaw), sinyaw = sin(actualYaw);
    actual_state->vx = vx_global * cosyaw + vy_global * sinyaw;
    actual_state->vy = -vx_global * sinyaw + vy_global * cosyaw;

    s->past_x = x; s->past_y = y; s->past_time = current_time;

    if (x_global) *x_global = x;
    if (y_global) *y_global = y;
    if (dt) *dt = local_dt;
}

double sensors_get_yaw(const Sensors *s){
    return s->last_yaw;
}
