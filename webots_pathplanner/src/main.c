/*
 * main.c - Main robot loop
 *
 * 功能：应用入口，初始化设备/模块，选择运行模式（手动或自动），并在主循环中调用传感器、规划、控制与执行器模块。
 */
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <webots/camera.h>
#include <webots/distance_sensor.h>
#include <webots/gps.h>
#include <webots/gyro.h>
#include <webots/inertial_unit.h>
#include <webots/keyboard.h>
#include <webots/motor.h>
#include <webots/robot.h>
#include "pid_controller.h"

#include "path_planner.h"
#include "devices.h"
#include "sensors.h"
#include "actuators.h"
#include "control.h"
#include "teleop.h"
#include "planner.h"
#include "nav.h"
#include "utils.h"
#define TIME_STEP 32
#define FLYING_ALTITUDE 0.2


int main(int argc, char **argv)
{
    wb_robot_init();
    const int timestep = (int)wb_robot_get_basic_time_step();

    // 设备初始化（封装到 devices.c）
    Devices dev;
    devices_init(timestep, &dev);

    // 传感器/执行器封装
    Sensors sensors;
    sensors_init(&sensors, &dev);

    Actuators actuators;
    actuators_init(&actuators, &dev);

    // ---------- PID 初始化 ----------
    actual_state_t actual_state = {0};
    desired_state_t desired_state = {0};
    motor_power_t motor_power = {0};

    gains_pid_t gains_pid = {
        .kp_att_y = 1, .kd_att_y = 0.5,
        .kp_att_rp = 0.5, .kd_att_rp = 0.1,
        .kp_vel_xy = 2, .kd_vel_xy = 0.5,
        .kp_z = 10, .ki_z = 5, .kd_z = 5
    };

    // 初始化控制模块（包含 PID controller 初始化）
    control_init(&gains_pid);

    double height_desired = FLYING_ALTITUDE;


    // =============== 模式选择 ===============
    int mode;     // 1：键盘手动   2：自动路径规划
    printf("控制模式：1=键盘，2=自动路径\n");
    scanf("%d", &mode);  // 用户可选择模式

    // ---------- 平滑器参数 ----------
    double smoothed_vx = 0.0, smoothed_vy = 0.0;
    double smoothed_alt = height_desired;
    const double max_accel = 0.8;      // 最大横向加速度 (m/s^2)
    const double max_climb_rate = 0.6; // 最大爬升速度  (m/s)

    // ---------- 主循环 ----------
    while (wb_robot_step(timestep) != -1)
    {
        // 更新传感器读数与 dt
        double x_global=0.0, y_global=0.0;
        double current_time = wb_robot_get_time();
        double current_dt = current_time - sensors.past_time;
        if (current_dt <= 0.0) current_dt = (double)timestep / 1000.0;
        sensors_update(&sensors, &actual_state, &x_global, &y_global, &current_dt);

        // ---------- 默认期望值 ----------
        desired_state.roll = 0;
        desired_state.pitch = 0;
        desired_state.yaw_rate = 0;
        desired_state.vx = 0;
        desired_state.vy = 0;
        desired_state.altitude = smoothed_alt;

        // ====================================================
        // -------------------- 键盘模式 ----------------------
        // ====================================================
        if (mode == 1)
        {
            teleop_handle(&desired_state, &smoothed_vx, &smoothed_vy, &smoothed_alt, current_dt);
        }  

        // ====================================================
        // -------------------- 路径规划 ------------------
        // ====================================================
        else if (mode == 2){
            static int planner_inited_local = 0;
            double goal[4] = {1.5, 1.5, 0.2, 0.0};
            if (!planner_inited_local) {
                planner_start(dev.gps, dev.imu, goal, 4);
                planner_inited_local = 1;
            }   

        // 更新导航（planner 包含 nav 并会计算 yaw_rate）
        double current_yaw = sensors_get_yaw(&sensors);
        planner_step(x_global, y_global, actual_state.altitude, current_yaw, &desired_state, current_dt);
    }

        // 控制器计算
        control_compute(&actual_state, &desired_state, &gains_pid, current_dt, &motor_power);

        // 应用马达输出
        actuators_apply(&actuators, &motor_power);

    }

    wb_robot_cleanup();
    return 0;
}
