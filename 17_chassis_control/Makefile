# Source selection follows the existing Keil project.
PROJECT := chassis_task
STACK_SIZE := 0x2000
HEAP_SIZE := 0x2000

DEFINES := \
	-DUSE_HAL_DRIVER \
	-DSTM32F407xx \
	-DARM_MATH_CM4 \
	-D__FPU_PRESENT=1U \
	-DARM_MATH_MATRIX_CHECK \
	-DARM_MATH_ROUNDING \
	-D__packed=

INCLUDES := \
	-IInc \
	-IDrivers/STM32F4xx_HAL_Driver/Inc \
	-IDrivers/STM32F4xx_HAL_Driver/Inc/Legacy \
	-IMiddlewares/Third_Party/FreeRTOS/Source/include \
	-IMiddlewares/Third_Party/FreeRTOS/Source/CMSIS_RTOS \
	-I../build_support/FreeRTOS/ARM_CM4F \
	-IDrivers/CMSIS/Device/ST/STM32F4xx/Include \
	-IDrivers/CMSIS/Include \
	-Ibsp/boards \
	-Iapplication \
	-Icomponents/algorithm \
	-Icomponents/algorithm/Include \
	-Icomponents/controller \
	-Icomponents/devices \
	-Icomponents/support

C_SOURCES := \
	Src/main.c \
	Src/gpio.c \
	Src/can.c \
	Src/dma.c \
	Src/freertos.c \
	Src/i2c.c \
	Src/rtc.c \
	Src/spi.c \
	Src/tim.c \
	Src/usart.c \
	Src/stm32f4xx_it.c \
	Src/stm32f4xx_hal_msp.c \
	Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_can.c \
	Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_i2c.c \
	Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_i2c_ex.c \
	Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_rtc.c \
	Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_rtc_ex.c \
	Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_spi.c \
	Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_tim.c \
	Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_tim_ex.c \
	Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_uart.c \
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
	Middlewares/Third_Party/FreeRTOS/Source/croutine.c \
	Middlewares/Third_Party/FreeRTOS/Source/event_groups.c \
	Middlewares/Third_Party/FreeRTOS/Source/list.c \
	Middlewares/Third_Party/FreeRTOS/Source/queue.c \
	Middlewares/Third_Party/FreeRTOS/Source/stream_buffer.c \
	Middlewares/Third_Party/FreeRTOS/Source/tasks.c \
	Middlewares/Third_Party/FreeRTOS/Source/timers.c \
	Middlewares/Third_Party/FreeRTOS/Source/CMSIS_RTOS/cmsis_os.c \
	Middlewares/Third_Party/FreeRTOS/Source/portable/MemMang/heap_4.c \
	bsp/boards/bsp_buzzer.c \
	bsp/boards/bsp_can.c \
	bsp/boards/bsp_delay.c \
	bsp/boards/bsp_flash.c \
	bsp/boards/bsp_imu_pwm.c \
	bsp/boards/bsp_led.c \
	bsp/boards/bsp_rc.c \
	bsp/boards/bsp_rng.c \
	bsp/boards/bsp_spi.c \
	bsp/boards/bsp_usart.c \
	application/calibrate_task.c \
	application/CAN_receive.c \
	application/control_demo_adapters.c \
	application/chassis_behaviour.c \
	application/chassis_power_control.c \
	application/chassis_task.c \
	application/detect_task.c \
	application/INS_task.c \
	application/led_flow_task.c \
	application/remote_control.c \
	application/test_task.c \
	components/devices/BMI088driver.c \
	components/devices/BMI088Middleware.c \
	components/devices/ist8310driver.c \
	components/devices/ist8310driver_middleware.c \
	components/algorithm/AHRS_middleware.c \
	components/algorithm/user_lib.c \
	components/support/CRC8_CRC16.c \
	components/controller/pid.c

COMMON_C_SOURCES := \
	FreeRTOS/ARM_CM4F/port.c \
	CMSIS-DSP/arm_sin_f32.c \
	CMSIS-DSP/arm_cos_f32.c \
	CMSIS-DSP/arm_common_tables.c

PREBUILT_LIBS := \
	components/algorithm/AHRS.lib

include ../build_support/common.mk
