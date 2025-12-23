/*
 * sensors.h - Sensor interface
 *
 * 声明传感器结构体与读取/更新接口。
 */
#ifndef SENSORS_H
#define SENSORS_H

#include <webots/inertial_unit.h>
#include <webots/gps.h>
#include <webots/gyro.h>
#include "pid_controller.h"
#include "devices.h"

typedef struct {
    WbDeviceTag imu;
    WbDeviceTag gps;
    WbDeviceTag gyro;
    double past_x;
    double past_y;
    double past_time;
    double last_yaw;
} Sensors;

void sensors_init(Sensors *s, Devices *dev);
void sensors_update(Sensors *s, actual_state_t *actual_state, double *x_global, double *y_global, double *dt);

// 获取最近一次由 IMU 提供的航向角（弧度）
double sensors_get_yaw(const Sensors *s);

#endif // SENSORS_H
