/*
 * utils.c - Utility helpers
 *
 * 功能：包含常用小工具函数（如角度规范化等）。
 */
#include "utils.h"
#include <math.h>

static inline double wrapToPi(double ang){
    while (ang > M_PI) ang -= 2.0*M_PI;
    while (ang <= -M_PI) ang += 2.0*M_PI;
    return ang;
}
double wrap_to_pi(double ang){
    return wrapToPi(ang);
}
