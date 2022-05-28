################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (10.3-2021.10)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../DC_Motor/DC_Motor.c 

OBJS += \
./DC_Motor/DC_Motor.o 

C_DEPS += \
./DC_Motor/DC_Motor.d 


# Each subdirectory must supply rules for building sources it contributes
DC_Motor/%.o DC_Motor/%.su: ../DC_Motor/%.c DC_Motor/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m0plus -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32G030xx -c -I../Core/Inc -I../Drivers/STM32G0xx_HAL_Driver/Inc -I../Drivers/STM32G0xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32G0xx/Include -I../Drivers/CMSIS/Include -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfloat-abi=soft -mthumb -o "$@"

clean: clean-DC_Motor

clean-DC_Motor:
	-$(RM) ./DC_Motor/DC_Motor.d ./DC_Motor/DC_Motor.o ./DC_Motor/DC_Motor.su

.PHONY: clean-DC_Motor

