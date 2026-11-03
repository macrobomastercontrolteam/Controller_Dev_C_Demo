#include "chassis_task.h"
#include "custom_ui_task.h"
#include "cv_usart_task.h"
#include "referee.h"
#include "shoot.h"

chassis_move_t chassis_move;
tCvCmdHandler CvCmdHandler;
ui_info_t ui_info;
ext_game_robot_state_t robot_state = {
    .power_management_gimbal_output = 1,
    .power_management_chassis_output = 1,
    .power_management_shooter_output = 1,
};
shoot_control_t shoot_control;

uint8_t is_game_started(void)
{
    return 1;
}

void shoot_init(void)
{
}

int16_t shoot_control_loop(void)
{
    return 0;
}

bool_t fCvAutoAim(void)
{
    return 0;
}
