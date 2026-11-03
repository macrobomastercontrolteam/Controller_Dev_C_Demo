#ifndef IMU_TEMP_TASK_H
#define IMU_TEMP_TASK_H


#include "struct_typedef.h"



/**
  * @brief          bmi088 temperature control 
  * @param[in]      argument: NULL
  * @retval         none
  */
/**
  * @brief          Control the BMI088 temperature.
  * @param[in]      argument: NULL
  * @retval         none
  */
extern void imu_temp_control_task(void const * argument);

#endif



