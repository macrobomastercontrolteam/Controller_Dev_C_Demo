/**
  ****************************(C) COPYRIGHT 2019 DJI****************************
  * @file       IST8310driver_middleware.c/h
  * @brief      the file provide I2C write/read function, as the middleware of IST8310.
  *             This file provides I2C read/write functions as middleware for the IST8310 driver.
  * @note       The IST8310 supports I2C only.
  * @history
  *  Version    Date            Author          Modification
  *  V1.0.0     Dec-26-2018     RM              1. done
  *
  @verbatim
  ==============================================================================

  ==============================================================================
  @endverbatim
  ****************************(C) COPYRIGHT 2019 DJI****************************
  */
#ifndef IST8310DRIVER_MIDDLEWARE_H
#define IST8310DRIVER_MIDDLEWARE_H

#include "struct_typedef.h"

#define IST8310_IIC_ADDRESS 0x0E  //the I2C address of IST8310


/**
  * @brief          initialize ist8310 gpio.
  * @param[in]      none
  * @retval         none
  */
/**
  * @brief          Initialize the IST8310 GPIO.
  * @param[in]      none
  * @retval         none
  */
extern void ist8310_GPIO_init(void);

/**
  * @brief          initialize ist8310 communication interface
  * @param[in]      none
  * @retval         none
  */
/**
  * @brief          Initialize the IST8310 communication interface.
  * @param[in]      none
  * @retval         none
  */
extern void ist8310_com_init(void);


/**
  * @brief          read a byte of ist8310 by i2c
  * @param[in]      register address
  * @retval         value of the register
  */
/**
  * @brief          Read one byte from the IST8310 over I2C.
  * @param[in]      Register address
  * @retval         Register value
  */
extern uint8_t ist8310_IIC_read_single_reg(uint8_t reg);

/**
  * @brief          write a byte of ist8310 by i2c
  * @param[in]      register address
  * @param[in]      write value
  * @retval         value of the register
  */
/**
  * @brief          Write one byte to an IST8310 register over I2C.
  * @param[in]      Register address
  * @param[in]      Value to write
  * @retval         none
  */
extern void ist8310_IIC_write_single_reg(uint8_t reg, uint8_t data);

/**
  * @brief          read multiple byte of ist8310 by i2c
  * @param[in]      register start address
  * @param[out]     read buffer
  * @param[in]      Size Amount of data to be read
  * @retval         none
  */
/**
  * @brief          Read multiple bytes from the IST8310 over I2C.
  * @param[in]      Starting register address
  * @param[out]     Data buffer
  * @param[in]      Number of bytes to read
  * @retval         none
  */
extern void ist8310_IIC_read_muli_reg(uint8_t reg, uint8_t *buf, uint8_t len);

/**
  * @brief          write multiple byte of ist8310 by i2c
  * @param[in]      register address
  * @param[out]     write buffer
  * @param[in]      Size Amount of data to be sent
  * @retval         none
  */
/**
  * @brief          Write multiple bytes to IST8310 registers over I2C.
  * @param[in]      Starting register address
  * @param[out]     Data buffer
  * @param[in]      Number of bytes to read
  * @retval         none
  */
extern void ist8310_IIC_write_muli_reg(uint8_t reg, uint8_t *data, uint8_t len);

/**
  * @brief          delay x millisecond
  * @param[in]      ms: ms millisecond
  * @retval         none
  */
/**
  * @brief          Delay for the specified number of milliseconds.
  * @param[in]      ms: delay in milliseconds
  * @retval         none
  */
extern void ist8310_delay_ms(uint16_t ms);
/**
  * @brief          delay x microsecond
  * @param[in]      us: us microsecond
  * @retval         none
  */
/**
  * @brief          Delay for the specified number of microseconds.
  * @param[in]      us: delay in microseconds
  * @retval         none
  */
extern void ist8310_delay_us(uint16_t us);
/**
  * @brief          set the RSTN PIN to 1
  * @param[in]      none
  * @retval         none
  */
/**
  * @brief          Set the RSTN pin high.
  * @param[in]      none
  * @retval         none
  */
extern void ist8310_RST_H(void);
/**
  * @brief          set the RSTN PIN to 0
  * @param[in]      none
  * @retval         none
  */
/**
  * @brief          Set the RSTN pin low.
  * @param[in]      none
  * @retval         none
  */
extern void ist8310_RST_L(void);

#endif
