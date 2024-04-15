################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (9-2020-q2-update)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../App/Src/app_master.c \
../App/Src/atcmd.c \
../App/Src/decimation_filter.c \
../App/Src/fft_processing.c \
../App/Src/lora_driver.c \
../App/Src/master_app.c \
../App/Src/time_domain_statistics.c 

OBJS += \
./App/Src/app_master.o \
./App/Src/atcmd.o \
./App/Src/decimation_filter.o \
./App/Src/fft_processing.o \
./App/Src/lora_driver.o \
./App/Src/master_app.o \
./App/Src/time_domain_statistics.o 

C_DEPS += \
./App/Src/app_master.d \
./App/Src/atcmd.d \
./App/Src/decimation_filter.d \
./App/Src/fft_processing.d \
./App/Src/lora_driver.d \
./App/Src/master_app.d \
./App/Src/time_domain_statistics.d 


# Each subdirectory must supply rules for building sources it contributes
App/Src/%.o: ../App/Src/%.c App/Src/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DARM_MATH_CM4 -DUSE_HAL_DRIVER -DSTM32L462xx -c -I"D:/EVKs/SensorBoard/TestingRev@/MurataVibrationSensorNode_V2.0/MurataVibrationSensorNode_V2.0/Utilities/lpm/tiny_lpm" -I../Core/Inc -I"D:/EVKs/SensorBoard/TestingRev@/MurataVibrationSensorNode_V2.0/MurataVibrationSensorNode_V2.0/Utilities/conf" -I"D:/EVKs/SensorBoard/TestingRev@/MurataVibrationSensorNode_V2.0/MurataVibrationSensorNode_V2.0/Utilities/misc" -I"D:/EVKs/SensorBoard/TestingRev@/MurataVibrationSensorNode_V2.0/MurataVibrationSensorNode_V2.0/Utilities/timer" -I../App/Inc -I../Drivers/CMSIS/DSP/Include -I../Drivers/STM32L4xx_HAL_Driver/Inc -I../Drivers/STM32L4xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32L4xx/Include -I../Drivers/CMSIS/Include -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

