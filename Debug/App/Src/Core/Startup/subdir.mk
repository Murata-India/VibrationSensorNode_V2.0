################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (9-2020-q2-update)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
S_SRCS += \
../App/Src/Core/Startup/startup_stm32l462ceux.s 

OBJS += \
./App/Src/Core/Startup/startup_stm32l462ceux.o 

S_DEPS += \
./App/Src/Core/Startup/startup_stm32l462ceux.d 


# Each subdirectory must supply rules for building sources it contributes
App/Src/Core/Startup/%.o: ../App/Src/Core/Startup/%.s App/Src/Core/Startup/subdir.mk
	arm-none-eabi-gcc -mcpu=cortex-m4 -g3 -DDEBUG -c -x assembler-with-cpp -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@" "$<"

