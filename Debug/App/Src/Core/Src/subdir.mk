################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (9-2020-q2-update)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../App/Src/Core/Src/adc_if.c \
../App/Src/Core/Src/main.c \
../App/Src/Core/Src/retarget.c \
../App/Src/Core/Src/rtc_if.c \
../App/Src/Core/Src/stm32_lpm_if.c \
../App/Src/Core/Src/stm32l4xx_hal_msp.c \
../App/Src/Core/Src/stm32l4xx_it.c \
../App/Src/Core/Src/sys_app.c \
../App/Src/Core/Src/sys_debug.c \
../App/Src/Core/Src/sysmem.c \
../App/Src/Core/Src/system_stm32l4xx.c \
../App/Src/Core/Src/usart.c 

OBJS += \
./App/Src/Core/Src/adc_if.o \
./App/Src/Core/Src/main.o \
./App/Src/Core/Src/retarget.o \
./App/Src/Core/Src/rtc_if.o \
./App/Src/Core/Src/stm32_lpm_if.o \
./App/Src/Core/Src/stm32l4xx_hal_msp.o \
./App/Src/Core/Src/stm32l4xx_it.o \
./App/Src/Core/Src/sys_app.o \
./App/Src/Core/Src/sys_debug.o \
./App/Src/Core/Src/sysmem.o \
./App/Src/Core/Src/system_stm32l4xx.o \
./App/Src/Core/Src/usart.o 

C_DEPS += \
./App/Src/Core/Src/adc_if.d \
./App/Src/Core/Src/main.d \
./App/Src/Core/Src/retarget.d \
./App/Src/Core/Src/rtc_if.d \
./App/Src/Core/Src/stm32_lpm_if.d \
./App/Src/Core/Src/stm32l4xx_hal_msp.d \
./App/Src/Core/Src/stm32l4xx_it.d \
./App/Src/Core/Src/sys_app.d \
./App/Src/Core/Src/sys_debug.d \
./App/Src/Core/Src/sysmem.d \
./App/Src/Core/Src/system_stm32l4xx.d \
./App/Src/Core/Src/usart.d 


# Each subdirectory must supply rules for building sources it contributes
App/Src/Core/Src/%.o: ../App/Src/Core/Src/%.c App/Src/Core/Src/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DARM_MATH_CM4 -DUSE_HAL_DRIVER -DSTM32L462xx -c -I"C:/Users/MIEB2001/STM32CubeIDE/workspace_1.7.0/MurataVibrationSensorNode_V2.0/Utilities/lpm/tiny_lpm" -I../Core/Inc -I"C:/Users/MIEB2001/STM32CubeIDE/workspace_1.7.0/MurataVibrationSensorNode_V2.0/Utilities/conf" -I"C:/Users/MIEB2001/STM32CubeIDE/workspace_1.7.0/MurataVibrationSensorNode_V2.0/Utilities/misc" -I"C:/Users/MIEB2001/STM32CubeIDE/workspace_1.7.0/MurataVibrationSensorNode_V2.0/Utilities/timer" -I../App/Inc -I../Drivers/CMSIS/DSP/Include -I../Drivers/STM32L4xx_HAL_Driver/Inc -I../Drivers/STM32L4xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32L4xx/Include -I../Drivers/CMSIS/Include -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

