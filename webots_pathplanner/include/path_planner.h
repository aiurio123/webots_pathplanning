#ifndef PATH_PLANNER_H
#define PATH_PLANNER_H

#include <webots/gps.h>
#include <webots/inertial_unit.h>
#include <webots/lidar.h>

#define MAX_WAYPOINTS 400

extern double waypoints[MAX_WAYPOINTS][4];
extern int waypoint_count;

/**
 * 初始化全局路径规划
 * gps   : GPS设备标签
 * imu   : IMU设备标签
 * lidar : LIDAR设备标签（可 NULL）
 * goal  : 目标点 [x,y,z,yaw]
 * segments : 将 D* Lite 粗路径分段数
 */
void init_global_path(WbDeviceTag gps, WbDeviceTag imu, double goal[4], int segments);

#endif
