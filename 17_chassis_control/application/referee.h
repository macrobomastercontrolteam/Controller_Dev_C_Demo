#ifndef REFEREE_H
#define REFEREE_H

#include "global_inc.h"

typedef struct
{
    uint8_t power_management_gimbal_output;
    uint8_t power_management_chassis_output;
    uint8_t power_management_shooter_output;
} ext_game_robot_state_t;

extern ext_game_robot_state_t robot_state;
uint8_t is_game_started(void);

#endif
