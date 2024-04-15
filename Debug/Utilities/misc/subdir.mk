################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (9-2020-q2-update)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Utilities/misc/stm32_mem.c \
../Utilities/misc/stm32_systime.c \
../Utilities/misc/stm32_tiny_sscanf.c \
../Utilities/misc/stm32_tiny_vsnprintf.c 

OBJS += \
./Utilities/misc/stm32_mem.o \
./Utilities/misc/stm32_systime.o \
./Utilities/misc/stm32_tiny_sscanf.o \
./Utilities/misc/stm32_tiny_vsnprintf.o 

C_DEPS += \
./Utilities/misc/stm32_mem.d \
./Utilities/misc/stm32_systime.d \
./Utilities/misc/stm32_tiny_sscanf.d \
./Utilities/misc/stm32_tiny_vsnprintf.d 


# Each subdirectory must supply rules for building sources it contributes
Utilities/misc/%.o: ../Utilities/misc/%.c Utilities/misc/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DARM_MATH_CM4 -DUSE_HAL_DRIVER -DSTM32L462xx -c -I"D:/EVKs/SensorBoard/TestingRev@/MurataVibrationSensorNode_V2.0/MurataVibrationSensorNode_V2.0/Utilities/lpm/tiny_lpm" -I../Core/Inc -I"D:/EVKs/SensorBoard/TestingRev@/MurataVibrationSensorNode_V2.0/MurataVibrationSensorNode_V2.0/Utilities/conf" -I"D:/EVKs/SensorBoard/TestingRev@/MurataVibrationSensorNode_V2.0/MurataVibrationSensorNode_V2.0/Utilities/misc" -I"D:/EVKs/SensorBoard/TestingRev@/MurataVibrationSensorNode_V2.0/MurataVibrationSensorNode_V2.0/Utilities/timer" -I../App/Inc -I../Drivers/CMSIS/DSP/Include -I../Drivers/STM32L4xx_HAL_Driver/Inc -I../Drivers/STM32L4xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32L4xx/Include -I../Drivers/CMSIS/Include -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

