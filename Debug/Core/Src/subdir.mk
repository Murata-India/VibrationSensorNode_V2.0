################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (9-2020-q2-update)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Core/Src/adc_if.c \
../Core/Src/main.c \
../Core/Src/retarget.c \
../Core/Src/rtc_if.c \
../Core/Src/stm32_lpm_if.c \
../Core/Src/stm32l4xx_hal_msp.c \
../Core/Src/stm32l4xx_it.c \
../Core/Src/sys_app.c \
../Core/Src/sys_debug.c \
../Core/Src/sysmem.c \
../Core/Src/system_stm32l4xx.c \
../Core/Src/usart.c 

OBJS += \
./Core/Src/adc_if.o \
./Core/Src/main.o \
./Core/Src/retarget.o \
./Core/Src/rtc_if.o \
./Core/Src/stm32_lpm_if.o \
./Core/Src/stm32l4xx_hal_msp.o \
./Core/Src/stm32l4xx_it.o \
./Core/Src/sys_app.o \
./Core/Src/sys_debug.o \
./Core/Src/sysmem.o \
./Core/Src/system_stm32l4xx.o \
./Core/Src/usart.o 

C_DEPS += \
./Core/Src/adc_if.d \
./Core/Src/main.d \
./Core/Src/retarget.d \
./Core/Src/rtc_if.d \
./Core/Src/stm32_lpm_if.d \
./Core/Src/stm32l4xx_hal_msp.d \
./Core/Src/stm32l4xx_it.d \
./Core/Src/sys_app.d \
./Core/Src/sys_debug.d \
./Core/Src/sysmem.d \
./Core/Src/system_stm32l4xx.d \
./Core/Src/usart.d 


# Each subdirectory must supply rules for building sources it contributes
Core/Src/%.o: ../Core/Src/%.c Core/Src/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DARM_MATH_CM4 -DUSE_HAL_DRIVER -DSTM32L462xx -c -I"D:/EVKs/SensorBoard/TestingRev@/MurataVibrationSensorNode_V2.0/MurataVibrationSensorNode_V2.0/Utilities/lpm/tiny_lpm" -I../Core/Inc -I"D:/EVKs/SensorBoard/TestingRev@/MurataVibrationSensorNode_V2.0/MurataVibrationSensorNode_V2.0/Utilities/conf" -I"D:/EVKs/SensorBoard/TestingRev@/MurataVibrationSensorNode_V2.0/MurataVibrationSensorNode_V2.0/Utilities/misc" -I"D:/EVKs/SensorBoard/TestingRev@/MurataVibrationSensorNode_V2.0/MurataVibrationSensorNode_V2.0/Utilities/timer" -I../App/Inc -I../Drivers/CMSIS/DSP/Include -I../Drivers/STM32L4xx_HAL_Driver/Inc -I../Drivers/STM32L4xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32L4xx/Include -I../Drivers/CMSIS/Include -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

