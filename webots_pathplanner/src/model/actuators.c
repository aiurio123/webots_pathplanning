/*
 * actuators.c - Actuator wrapper
 *
 * 功能：封装飞控的电机接口（四个电机），提供初始化与下发电机速度的函数。
 */
#include "actuators.h"
#include <webots/motor.h>

void actuators_init(Actuators *a, Devices *dev){
    a->m1 = dev->m1; a->m2 = dev->m2; a->m3 = dev->m3; a->m4 = dev->m4;
}

void actuators_apply(const Actuators *a, const motor_power_t *power){
    wb_motor_set_velocity(a->m1, -power->m1);
    wb_motor_set_velocity(a->m2,  power->m2);
    wb_motor_set_velocity(a->m3, -power->m3);
    wb_motor_set_velocity(a->m4,  power->m4);
}
