/**
  ****************************(C) COPYRIGHT 2019 DJI****************************
  * @file       chassis.c/h
  * @brief      chassis control task,
  * @note
  * @history
  *  Version    Date            Author          Modification
  *  V1.0.0     Dec-26-2018     RM              1. finished
  *  V1.1.0     Nov-11-2019     RM              1. add chassis power control
  *
  @verbatim
  ==============================================================================

  ==============================================================================
  @endverbatim
  ****************************(C) COPYRIGHT 2019 DJI****************************
  */
#ifndef CHASSIS_TASK_H
#define CHASSIS_TASK_H
#include "CAN_receive.h"
#include "gimbal_task.h"
#include "global_inc.h"
#include "pid.h"
#include "remote_control.h"
#include "user_lib.h"

// default values
#define SPINNING_CHASSIS_MAX_OMEGA RPM_TO_RADS (240.0f)
#define SPINNING_CHASSIS_HIGH_OMEGA (SPINNING_CHASSIS_MAX_OMEGA * 0.833f)
#define SPINNING_CHASSIS_MED_OMEGA (SPINNING_CHASSIS_MAX_OMEGA * 0.667f)
#define SPINNING_CHASSIS_LOW_OMEGA (SPINNING_CHASSIS_MAX_OMEGA * 0.583f)
#define SPINNING_CHASSIS_ULTRA_LOW_OMEGA (SPINNING_CHASSIS_MAX_OMEGA * 0.167f)

// in the beginning of task ,wait a time
#define CHASSIS_TASK_INIT_TIME 357

#define CHASSIS_ACCEL_WZ_NUM 0.06f
#define CHASSIS_ACCEL_X_NUM 3.0f
#define CHASSIS_ACCEL_Y_NUM 3.0f
#define CHASSIS_DECEL_X_NUM (CHASSIS_ACCEL_X_NUM * 20.0f) // stop in 1/4 of acceleration time
#define CHASSIS_DECEL_Y_NUM (CHASSIS_ACCEL_Y_NUM * 20.0f)

// joystick value deadline
#define CHASSIS_RC_DEADLINE 20

#define CHASSIS_TEST_MODE 0


#if (ROBOT_TYPE == SENTRY_2026_OMNI) || (ROBOT_TYPE == INFANTRY_2026_OMNI)
#define MOTOR_DISTANCE_TO_CENTER_DEFAULT 0.15f
#else
#define MOTOR_DISTANCE_TO_CENTER_DEFAULT 0.277205f
#endif

#define CHASSIS_CONTROL_TIME_MS 5.0f
#define CHASSIS_CONTROL_TIME_S (CHASSIS_CONTROL_TIME_MS / 1000.0f)
#define CHASSIS_CONTROL_FREQUENCE (1.0f / CHASSIS_CONTROL_TIME_S)

// chassis 3508 max motor control current
#define MAX_3508_MOTOR_CAN_CURRENT 16000.0f
// chassis 6020 max motor control voltage
#define MAX_MOTOR_CAN_VOLTAGE 20000.0f
// // press the key, chassis will swing
// #define SWING_KEY KEY_PRESSED_OFFSET_CTRL
// chassi forward, back, left, right key
#define CHASSIS_FRONT_KEY KEY_PRESSED_OFFSET_W
#define CHASSIS_BACK_KEY KEY_PRESSED_OFFSET_S
#define CHASSIS_LEFT_KEY KEY_PRESSED_OFFSET_A
#define CHASSIS_RIGHT_KEY KEY_PRESSED_OFFSET_D

// drive wheel parameters
#define M3508_MOTOR_GEAR_RATIO (3591.0f / 187.0f)
#define DRIVE_WHEEL_RADIUS 0.0785f
// Ratio of M3508 speed in rpm to chassis speed in m/s
#if (MOTOR_TYPE == POWER_TRAIN_USE_4010_MOTOR)

#define MG4010_MOTOR_RPM_TO_VECTOR ((2.0f * PI / 60.0f) * DRIVE_WHEEL_RADIUS)
#define CHASSIS_MOTOR_RPM_TO_VECTOR_SEN MG4010_MOTOR_RPM_TO_VECTOR
#define MOTOR_ROTOR_TO_OUTPUT_CONSTANT  (1.0f / (2.0f * PI * DRIVE_WHEEL_RADIUS) * 360 * MOTOR_MG4010_GEAR_RATIO)
#else
#define M3508_MOTOR_RPM_TO_VECTOR ((2.0f * PI / 60.0f) * DRIVE_WHEEL_RADIUS / M3508_MOTOR_GEAR_RATIO)
#define CHASSIS_MOTOR_RPM_TO_VECTOR_SEN M3508_MOTOR_RPM_TO_VECTOR
#define MOTOR_ROTOR_TO_OUTPUT_CONSTANT 1.0f
#endif

// single chassis motor max speed
#define MAX_WHEEL_SPEED 3.0f
#if(ROBOT_TYPE == HERO_2025_MECANUM)
// chassis forward or back max speed
#define NORMAL_MAX_CHASSIS_SPEED_X 5.5f 
#define SPRINT_MAX_CHASSIS_SPEED_X 6.5f
#define LOW_POWER_MAX_CHASSIS_SPEED_X 0.25f 
// chassis left or right max speed
#define NORMAL_MAX_CHASSIS_SPEED_Y 3.5f
#define SPRINT_MAX_CHASSIS_SPEED_Y 4.5f
#define LOW_POWER_MAX_CHASSIS_SPEED_Y 0.1f 
#else
// chassis forward or back max speed
#define NORMAL_MAX_CHASSIS_SPEED_X 5.5f //Forward/Backward Direction
#define SPRINT_MAX_CHASSIS_SPEED_X 6.5f
#define LOW_POWER_MAX_CHASSIS_SPEED_X 0.25f 
// chassis left or right max speed
#define NORMAL_MAX_CHASSIS_SPEED_Y 4.5f //Sideway
#define SPRINT_MAX_CHASSIS_SPEED_Y 5.5f
#define LOW_POWER_MAX_CHASSIS_SPEED_Y 0.1f 
#endif
#define NORMAL_TO_SPRINT_MAX_CHASSIS_SPEED_RATIO 1.5f

#if (ROBOT_TYPE == HERO_2025_MECANUM)
#define MOTOR_SPEED_TO_CHASSIS_SPEED_VX 0.25f
#define MOTOR_SPEED_TO_CHASSIS_SPEED_VY 0.25f
#define MOTOR_SPEED_TO_CHASSIS_SPEED_WZ 0.25f
#else
// Omni wheel kinematics conversion factors
// Adjust these values based on your specific omni wheel configuration and chassis dimensions
#define MOTOR_SPEED_TO_CHASSIS_SPEED_VX 0.25f
#define MOTOR_SPEED_TO_CHASSIS_SPEED_VY 0.25f
#define MOTOR_SPEED_TO_CHASSIS_SPEED_WZ 2.0f
#endif

// In follow-yaw mode, map joystick value to increment in target yaw angle
#define CHASSIS_ANGLE_Z_RC_CHANGE_TIME_S 1.0f
#define CHASSIS_ANGLE_Z_RC_SEN_INC (PI / 2.0f / CHASSIS_ANGLE_Z_RC_CHANGE_TIME_S * CHASSIS_CONTROL_TIME_S / JOYSTICK_HALF_RANGE)

// Arbitrary offsets between chassis rotational center and centroid
// slip ring is at the center of chassis
#define CHASSIS_WZ_SET_SCALE 0.0f

// when chassis is not set to move, swing max angle
#define SWING_NO_MOVE_ANGLE 0.7f
// when chassis is set to move, swing max angle
#define SWING_MOVE_ANGLE 0.31415926535897932384626433832795f

// chassis motor speed PID
#if (ROBOT_TYPE == SENTRY_2026_OMNI) || (ROBOT_TYPE == INFANTRY_2026_OMNI)
#define MG4010_MOTOR_SPEED_PID_KP 1.0f
#define MG4010_MOTOR_SPEED_PID_KI 0.0f
#define MG4010_MOTOR_SPEED_PID_KD 0.0f
#define MG4010_MOTOR_SPEED_PID_MAX_OUT 3.0f
#define MG4010_MOTOR_SPEED_PID_MAX_IOUT 0.2f
#else
// @TODO: tune pid for other robots
// @TODO: fix drift in spinning mode when power is limited
#define M3508_MOTOR_SPEED_PID_KP 21000.0f
#define M3508_MOTOR_SPEED_PID_KI 0.0f
#define M3508_MOTOR_SPEED_PID_KD 0.0f
#define M3508_MOTOR_SPEED_PID_MAX_OUT MAX_3508_MOTOR_CAN_CURRENT
#define M3508_MOTOR_SPEED_PID_MAX_IOUT 2000.0f
#endif

// chassis follow angle PID
#define CHASSIS_FOLLOW_GIMBAL_PID_KP 12.0f
#define CHASSIS_FOLLOW_GIMBAL_PID_KI 0.0f
#define CHASSIS_FOLLOW_GIMBAL_PID_KD 0.0002f
#define CHASSIS_FOLLOW_GIMBAL_PID_MAX_OUT 10.0f
#define CHASSIS_FOLLOW_GIMBAL_PID_MAX_IOUT 0.2f

typedef enum
{
	CHASSIS_COORDINATE_FOLLOW_CHASSIS_ABSOLUTE_FRONT, // chassis movement follow absolute coordinate (base on the initial direction faced by camera)
	CHASSIS_COORDINATE_FOLLOW_GIMBAL,	// chassis movement follow gimbal coordinate (camera direction as front)
	CHASSIS_COORDINATE_FOLLOW_CHASSIS_RELATIVE_FRONT,	// chassis movement follow chassis coordinate (chassis front as front)
} chassis_coord_sys_e;

typedef struct
{
	const motor_measure_t *chassis_motor_measure;
	fp32 accel;
	fp32 speed;
	fp32 speed_set;
	int16_t give_chassis_motor_cmd; // control current command for chassis M3508 motor deg/s*(gear ratio) for MG4010 //in DJI motor CAN current, 16384 => 10 A 
} chassis_motor_t;


typedef union
{
	uint8_t can_buf[8];
	struct
	{
		// 0: not provide power
		// 1: provide power
		uint8_t cap_state;
		uint8_t reserve;
		uint16_t cap_milivoltage;
		float cap_power;
	} cap_message;
} supcap_t;

extern supcap_t cap_message_rx;


typedef struct
{
	const RC_ctrl_t *chassis_RC;               // the point to remote control
	const gimbal_motor_t *chassis_yaw_motor;   // will use the relative angle of yaw gimbal motor to calculate the euler angle
	const gimbal_motor_t *chassis_pitch_motor; // will use the relative angle of pitch gimbal motor to calculate the euler angle
	const fp32 *chassis_INS_angle;             // the point to the euler angle of gyro sensor
	chassis_coord_sys_e chassis_coord_sys;               // state machine
	pid_type_def chassis_angle_pid;            // follow angle PID

	fp32 wheel_rot_radii[4];
	chassis_motor_t motor_chassis[4];          // chassis motor data
	pid_type_def motor_speed_pid[4];           // motor speed PID

	ramp_function_source_t chassis_cmd_slow_set_vx;    // RC input ramp for vx command shaping
	ramp_function_source_t chassis_cmd_slow_set_vy;    // RC input ramp for vy command shaping
	first_order_filter_type_t chassis_cmd_slow_set_wz; // use first order filter to slow set-point

	fp32 vx; // chassis vertical speed, positive means forward,unit m/s
	fp32 vy; // chassis horizontal speed, positive means letf,unit m/s
	fp32 wz; // chassis rotation speed, positive means counterclockwise,unit rad/s
	fp32 vx_set;                     // chassis set vertical speed,positive means forward,unit m/s
	fp32 vy_set;                     // chassis set horizontal speed,positive means left,unit m/s
	fp32 wz_set;                     // chassis set rotation speed,positive means counterclockwise,unit rad/s
	fp32 chassis_relative_angle_set; // the set relative angle
	fp32 chassis_yaw_set;

	fp32 vx_max_speed; // max forward speed, unit m/s
	fp32 vy_max_speed; // max letf speed, unit m/s
	fp32 wz_max_speed; // max spinning speed, unit rad/s.

	fp32 chassis_yaw;   // the yaw angle calculated by gyro sensor and gimbal motor
	fp32 chassis_pitch; // the pitch angle calculated by gyro sensor and gimbal motor
	fp32 chassis_roll;  // the roll angle calculated by gyro sensor and gimbal motor

	uint8_t fRandomSpinOn;
	int16_t dial_channel_latched;
	int16_t dial_channel_out;

} chassis_move_t;

/**
 * @brief          chassis task, osDelay CHASSIS_CONTROL_TIME_MS (2ms)
 * @param[in]      pvParameters: null
 * @retval         none
 */
extern void chassis_task(void const *pvParameters);

/**
 * @brief          accroding to the channel value of remote control, calculate chassis vertical and horizontal speed set-point
 *
 * @param[out]     vx_set: vertical speed set-point
 * @param[out]     vy_set: horizontal speed set-point
 * @retval         none
 */
extern void chassis_rc_to_control_vector(fp32 *vx_set, fp32 *vy_set);

fp32 chassis_get_high_wz_limit(void);
fp32 chassis_get_med_wz_limit(void);
fp32 chassis_get_low_wz_limit(void);
fp32 chassis_get_ultra_low_wz_limit(void);

void get_chassis_vel_in_gimbal_frame(fp32 *vx, fp32 *vy);

fp32 calc_wz_max_speed(fp32 vx_speed_limit, fp32 vy_speed_limit, fp32 wz_scaling_factor);




extern chassis_move_t chassis_move;

#endif
