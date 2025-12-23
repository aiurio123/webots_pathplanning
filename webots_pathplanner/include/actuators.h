/*
 * actuators.h - Actuator interface
 *
 * 声明 Actuators 结构与提供对电机的初始化与应用接口。
 */
#ifndef ACTUATORS_H
#define ACTUATORS_H

#include <webots/motor.h>
#include "devices.h"
#include "pid_controller.h"

typedef struct {
    WbDeviceTag m1, m2, m3, m4;
} Actuators;

void actuators_init(Actuators *a, Devices *dev);
void actuators_apply(const Actuators *a, const motor_power_t *power);

#endif // ACTUATORS_H
