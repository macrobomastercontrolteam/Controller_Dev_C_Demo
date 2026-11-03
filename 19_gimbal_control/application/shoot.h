#ifndef SHOOT_H
#define SHOOT_H

#include "global_inc.h"

typedef struct
{
    int16_t fric1_given_current;
    int16_t fric2_given_current;
    int16_t fric3_given_current;
    int16_t fric4_given_current;
    uint16_t piston_given_current;
    fp32 friction_motor1_rpm;
    fp32 friction_motor2_rpm;
} shoot_control_t;

extern shoot_control_t shoot_control;
void shoot_init(void);
int16_t shoot_control_loop(void);
bool_t fCvAutoAim(void);

#endif
