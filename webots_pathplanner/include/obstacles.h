/*
 * obstacles.h - Obstacles definitions
 *
 * 定义障碍物数据结构与加载默认场景的接口。
 */
#ifndef OBSTACLES_H
#define OBSTACLES_H

typedef struct {
double x;
double y;
double z;
} Obstacle;

// 最大障碍数
#define MAX_OBSTACLES 64

extern Obstacle obstacles_list[MAX_OBSTACLES];
extern int obs_count;

// 初始化障碍（硬编码 / 未来也能改成从传感器）
void load_default_obstacles(void);

#endif
