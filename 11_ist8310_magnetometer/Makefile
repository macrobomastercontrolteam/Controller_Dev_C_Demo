# Source selection follows the existing Keil project.
PROJECT := ist8310
STACK_SIZE := 0x400
HEAP_SIZE := 0x200

DEFINES := \
	-DUSE_HAL_DRIVER \
	-DSTM32F407xx

INCLUDES := \
	-IInc \
	-IDrivers/STM32F4xx_HAL_Driver/Inc \
	-IDrivers/STM32F4xx_HAL_Driver/Inc/Legacy \
	-IDrivers/CMSIS/Device/ST/STM32F4xx/Include \
	-IDrivers/CMSIS/Include \
	-Ibsp/boards

C_SOURCES := \
	Src/main.c \
	Src/gpio.c \
	Src/i2c.c \
	Src/stm32f4xx_it.c \
	Src/stm32f4xx_hal_msp.c \
	Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_i2c.c \
	Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_i2c_ex.c \
	Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_tim.c \
	Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_tim_ex.c \
	Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_rcc.c \
	Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_rcc_ex.c \
	Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_flash.c \
	Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_flash_ex.c \
	Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_flash_ramfunc.c \
	Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_gpio.c \
	Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_dma_ex.c \
	Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_dma.c \
	Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_pwr.c \
	Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_pwr_ex.c \
	Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_cortex.c \
	Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal.c \
	Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_exti.c \
	Src/system_stm32f4xx.c \
	bsp/boards/ist8310driver.c \
	bsp/boards/ist8310driver_middleware.c

include ../build_support/common.mk
