/**
  ****************************(C) COPYRIGHT 2016 DJI****************************
  * @file       pid.c/h
  * @brief      PID implementation, including initialization and calculation.
  * @note       
  * @history
  *  Version    Date            Author          Modification
  *  V1.0.0     Dec-26-2018     RM              1. Completed.
  *
  @verbatim
  ==============================================================================

  ==============================================================================
  @endverbatim
  ****************************(C) COPYRIGHT 2016 DJI****************************
  */
#ifndef PID_H
#define PID_H
#include "struct_typedef.h"
enum PID_MODE
{
    PID_POSITION = 0,
    PID_DELTA
};

typedef struct
{
    uint8_t mode;
    //Three PID gains.
    fp32 Kp;
    fp32 Ki;
    fp32 Kd;

    fp32 max_out;  //Maximum output.
    fp32 max_iout; //Maximum integral output.

    fp32 set;
    fp32 fdb;

    fp32 out;
    fp32 Pout;
    fp32 Iout;
    fp32 Dout;
    fp32 Dbuf[3];  //Derivative terms: 0 = latest, 1 = previous, 2 = two samples ago.
    fp32 error[3]; //Error terms: 0 = latest, 1 = previous, 2 = two samples ago.

} pid_type_def;
/**
  * @brief          pid struct data init
  * @param[out]     pid: PID struct data point
  * @param[in]      mode: PID_POSITION: normal pid
  *                 PID_DELTA: delta pid
  * @param[in]      PID: 0: kp, 1: ki, 2:kd
  * @param[in]      max_out: pid max out
  * @param[in]      max_iout: pid max iout
  * @retval         none
  */
/**
  * @brief          pid struct data init
  * @param[out]     pid: pointer to the PID data structure
  * @param[in]      mode: PID_POSITION: positional PID
  *                 PID_DELTA: incremental PID
  * @param[in]      PID: 0: kp, 1: ki, 2:kd
  * @param[in]      max_out: maximum PID output
  * @param[in]      max_iout: maximum PID integral output
  * @retval         none
  */
extern void PID_init(pid_type_def *pid, uint8_t mode, const fp32 PID[3], fp32 max_out, fp32 max_iout);

/**
  * @brief          pid calculate 
  * @param[out]     pid: PID struct data point
  * @param[in]      ref: feedback data 
  * @param[in]      set: set point
  * @retval         pid out
  */
/**
  * @brief          Calculate the PID output.
  * @param[out]     pid: pointer to the PID data structure
  * @param[in]      ref: feedback value
  * @param[in]      set: setpoint
  * @retval         PID output
  */
extern fp32 PID_calc(pid_type_def *pid, fp32 ref, fp32 set);

/**
  * @brief          pid out clear
  * @param[out]     pid: PID struct data point
  * @retval         none
  */
/**
  * @brief          Clear the PID output.
  * @param[out]     pid: pointer to the PID data structure
  * @retval         none
  */
extern void PID_clear(pid_type_def *pid);

#endif
