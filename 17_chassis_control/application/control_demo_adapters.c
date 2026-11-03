#include "chassis_task.h"
#include "control_demo_adapters.h"
#include "custom_ui_task.h"
#include "cv_usart_task.h"
#include "gimbal_behaviour.h"
#include "gimbal_task.h"
#include "referee.h"
#include "shoot.h"

gimbal_control_t gimbal_control;
tCvCmdHandler CvCmdHandler;
ui_info_t ui_info;
ext_game_robot_state_t robot_state = {
    .power_management_gimbal_output = 1,
    .power_management_chassis_output = 1,
    .power_management_shooter_output = 1,
};
shoot_control_t shoot_control;

void control_demo_adapters_init(void)
{
    can_ref_info.chassis_power_buffer = 60;
    can_ref_info.chassis_power_limit = 80;
    can_ref_info.PowerMeter_reading = 0.0f;
}

bool_t gimbal_cmd_to_chassis_stop(void)
{
    return 0;
}

const gimbal_motor_t *get_yaw_motor_point(void)
{
    return &gimbal_control.gimbal_yaw_motor;
}

const gimbal_motor_t *get_pitch_motor_point(void)
{
    return &gimbal_control.gimbal_pitch_motor;
}

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
