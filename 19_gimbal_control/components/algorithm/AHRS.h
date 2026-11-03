#ifndef AHRS_H
#define AHRS_H

#include "AHRS_MiddleWare.h"

/**
  * @brief          Initialize a quaternion using accelerometer and magnetometer data.
  * @param[in]      Quaternion array to initialize
  * @param[in]      Non-null accelerometer (x, y, z) data for initialization, in m/s2 
  * @param[in]      Non-null magnetometer (x, y, z) data for initialization, in uT
  * @retval         None
  */
extern void AHRS_init(fp32 quat[4], const fp32 accel[3], const fp32 mag[3]);

/**
  * @brief          Update a quaternion using gyroscope, accelerometer, and magnetometer data.
  * @param[in]      Quaternion array to update
  * @param[in]      Fixed update interval in seconds; for example, pass 0.001f for 1000 Hz
  * @param[in]      Gyroscope data in (x, y, z) order for the update, in rad/s
  * @param[in]      Accelerometer data in (x, y, z) order, in m/s2 
  * @param[in]      Magnetometer data in (x, y, z) order, in uT
  * @retval         1 on a successful update; 0 on failure
  */
extern bool_t AHRS_update(fp32 quat[4], const fp32 timing_time, const fp32 gyro[3], const fp32 accel[3], const fp32 mag[3]);

/**
  * @brief          Calculate the yaw Euler angle from a quaternion.
  * @param[in]      Non-null quaternion array
  * @retval         Yaw angle, in radians
  */
extern fp32 get_yaw(const fp32 quat[4]);

/**
  * @brief          Calculate the pitch Euler angle from a quaternion.
  * @param[in]      Non-null quaternion array
  * @retval         Pitch angle, in radians
  */
extern fp32 get_pitch(const fp32 quat[4]);
/**
  * @brief          Calculate the roll Euler angle from a quaternion.
  * @param[in]      Non-null quaternion array
  * @retval         Roll angle, in radians
  */
extern fp32 get_roll(const fp32 quat[4]);

/**
  * @brief          Calculate yaw, pitch, and roll Euler angles from a quaternion.
  * @param[in]      Non-null quaternion array
  * @param[in]      Returned yaw angle, in radians
  * @param[in]      Returned pitch angle, in radians
  * @param[in]      Returned roll angle, in radians
  */
extern void get_angle(const fp32 quat[4], fp32 *yaw, fp32 *pitch, fp32 *roll);
/**
  * @brief          Return the current gravitational acceleration.
  * @param[in]      None
  * @retval         Gravitational acceleration, in m/s2
  */
extern fp32 get_carrier_gravity(void);

#endif
