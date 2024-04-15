################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (9-2020-q2-update)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Utilities/lpm/tiny_lpm/stm32_lpm.c 

OBJS += \
./Utilities/lpm/tiny_lpm/stm32_lpm.o 

C_DEPS += \
./Utilities/lpm/tiny_lpm/stm32_lpm.d 


# Each subdirectory must supply rules for building sources it contributes
Utilities/lpm/tiny_lpm/%.o: ../Utilities/lpm/tiny_lpm/%.c Utilities/lpm/tiny_lpm/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DARM_MATH_CM4 -DUSE_HAL_DRIVER -DSTM32L462xx -c -I"D:/EVKs/SensorBoard/TestingRev@/MurataVibrationSensorNode_V2.0/MurataVibrationSensorNode_V2.0/Utilities/lpm/tiny_lpm" -I../Core/Inc -I"D:/EVKs/SensorBoard/TestingRev@/MurataVibrationSensorNode_V2.0/MurataVibrationSensorNode_V2.0/Utilities/conf" -I"D:/EVKs/SensorBoard/TestingRev@/MurataVibrationSensorNode_V2.0/MurataVibrationSensorNode_V2.0/Utilities/misc" -I"D:/EVKs/SensorBoard/TestingRev@/MurataVibrationSensorNode_V2.0/MurataVibrationSensorNode_V2.0/Utilities/timer" -I../App/Inc -I../Drivers/CMSIS/DSP/Include -I../Drivers/STM32L4xx_HAL_Driver/Inc -I../Drivers/STM32L4xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32L4xx/Include -I../Drivers/CMSIS/Include -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

