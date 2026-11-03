/**
  ****************************(C) COPYRIGHT 2016 DJI****************************
  * @file       IST8310middleware.c/h
  * @brief      IST8310 I2C middleware. Since the MPU6500 uses SPI,
  *             IST8310 reads use MPU6500 IIC_SLV0 and writes use IIC_SLV4.
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

#ifndef IST8310DRIVER_MIDDLEWARE_H
#define IST8310DRIVER_MIDDLEWARE_H

#include "struct_typedef.h"

#define IST8310_IIC_ADDRESS (0x0E << 1)  //IST8310 I2C address.
#define IST8310_IIC_READ_MSB (0x80) //Set the first bit for SPI register reads.

extern void ist8310_GPIO_init(void); //Initialize the IST8310 GPIO.
extern void ist8310_com_init(void);  //Initialize IST8310 communication.
extern uint8_t ist8310_IIC_read_single_reg(uint8_t reg);
extern void ist8310_IIC_write_single_reg(uint8_t reg, uint8_t data);
extern void ist8310_IIC_read_muli_reg(uint8_t reg, uint8_t *buf, uint8_t len);
extern void ist8310_IIC_write_muli_reg(uint8_t reg, uint8_t *data, uint8_t len);
extern void ist8310_delay_ms(uint16_t ms);
extern void ist8310_delay_us(uint16_t us);
extern void ist8310_RST_H(void); //Set the reset pin high.
extern void ist8310_RST_L(void); //Set the reset pin low to reset the IST8310.

#endif
