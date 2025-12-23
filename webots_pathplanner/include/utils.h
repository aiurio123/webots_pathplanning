/*
 * utils.h - Utility declarations
 *
 * 声明常用工具函数（如 wrap_to_pi）。
 */
#ifndef UTILS_H
#define UTILS_H

#include <math.h>

/* Normalize angle to (-pi, pi] */
double wrap_to_pi(double ang);

#endif // UTILS_H
