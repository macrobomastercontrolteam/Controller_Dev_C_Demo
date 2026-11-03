/**
  ****************************(C) COPYRIGHT 2016 DJI****************************
  * @file       IST8310.c/h
  * @brief      IST8310 magnetometer driver: initialization, data processing, and communication.
  *             This project configures MPU6500 IIC_SLV0 to read IST8310 data automatically.
  *             MPU_EXT_SENS_DATA_00 holds the IST8310 status; its flags determine when to update
  *             the data.
  * @note       The IST8310 supports I2C reads only.
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

#ifndef IST8310DRIVER_H
#define IST8310DRIVER_H

#include "struct_typedef.h"

#define IST8310_DATA_READY_BIT 2

#define IST8310_NO_ERROR 0x00

#define IST8310_NO_SENSOR 0x40

typedef struct ist8310_real_data_t
{
  uint8_t status;
  fp32 mag[3];
} ist8310_real_data_t;

extern uint8_t ist8310_init(void);
extern void ist8310_read_over(uint8_t *status_buf, ist8310_real_data_t *mpu6500_real_data);
extern void ist8310_read_mag(fp32 mag[3]);
#endif
