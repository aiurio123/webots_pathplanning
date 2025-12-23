/*
 * devices.h - Devices definitions
 *
 * 定义 Devices 结构体（包含电机与各类传感器的设备标签）并声明初始化接口。
 */
#ifndef DEVICES_H
#define DEVICES_H

#include <webots/robot.h>
#include <webots/motor.h>
#include <webots/inertial_unit.h>
#include <webots/gps.h>
#include <webots/gyro.h>
#include <webots/camera.h>
#include <webots/lidar.h>

typedef struct {
    WbDeviceTag m1, m2, m3, m4;
    WbDeviceTag imu, gps, gyro, camera, lidar;
} Devices;

void devices_init(int timestep, Devices *d);

#endif // DEVICES_H
