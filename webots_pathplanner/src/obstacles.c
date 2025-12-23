/*
 * obstacles.c - Obstacles data
 *
 * 功能：维护场景中障碍物列表（用于路径规划）。
 */
#include "obstacles.h"

// 全局障碍数组
Obstacle obstacles_list[MAX_OBSTACLES];
int obs_count = 0;

//手动设置障碍物三维坐标点
void load_default_obstacles(void)
{
obs_count = 8;
obstacles_list[0] = (Obstacle){0.1, 0.9, 0.2};
obstacles_list[1] = (Obstacle){0.7, 0.3, 0.2};
obstacles_list[2] = (Obstacle){1, 1, 0.2};
obstacles_list[3] = (Obstacle){0.5, 0.8, 0.2};

//   obstacles_list[obstacles_count++] = (Obstacle){...};

}
