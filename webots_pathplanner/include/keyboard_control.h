#ifndef KEYBOARD_CONTROL_H
#define KEYBOARD_CONTROL_H

#include <webots/keyboard.h>

typedef struct {
    double smoothed_vx;
    double smoothed_vy;
    double smoothed_alt;

    double max_accel;
    double max_climb_rate;
} KeyboardController;

// 对外接口
void keyboard_init(KeyboardController *kbd,
                   double max_accel,
                   double max_climb_rate,
                   double init_alt);

void keyboard_update(KeyboardController *kbd,
                     double dt,
                     void *desired_state); 

#endif
