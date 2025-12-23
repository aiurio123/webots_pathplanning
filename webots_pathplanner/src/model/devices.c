/*
 * devices.c - Hardware device initialization
 *
 * 功能：查找并初始化机器人设备（电机、传感器等），并启用所需的采样周期。
 */
#include "devices.h"
#include <webots/motor.h>
#include <webots/keyboard.h>
#include <stdio.h>

void devices_init(int timestep, Devices *d){
    //电机
    d->m1 = wb_robot_get_device("m1_motor");
    wb_motor_set_position(d->m1, INFINITY);
    wb_motor_set_velocity(d->m1, 0.0);

    d->m2 = wb_robot_get_device("m2_motor");
    wb_motor_set_position(d->m2, INFINITY);
    wb_motor_set_velocity(d->m2, 0.0);

    d->m3 = wb_robot_get_device("m3_motor");
    wb_motor_set_position(d->m3, INFINITY);
    wb_motor_set_velocity(d->m3, 0.0);

    d->m4 = wb_robot_get_device("m4_motor");
    wb_motor_set_position(d->m4, INFINITY);
    wb_motor_set_velocity(d->m4, 0.0);

    //传感器（你可以根据具体情况自行添加，本示例未用到camera与lidar）
    d->imu = wb_robot_get_device("inertial_unit");
    wb_inertial_unit_enable(d->imu, timestep);

    d->gps = wb_robot_get_device("gps");
    wb_gps_enable(d->gps, timestep);

    d->gyro = wb_robot_get_device("gyro");
    wb_gyro_enable(d->gyro, timestep);

    d->camera = wb_robot_get_device("camera");
    wb_camera_enable(d->camera, timestep);

    d->lidar = wb_robot_get_device("lidar");
    wb_lidar_enable(d->lidar, timestep);

    wb_keyboard_enable(timestep);
}
