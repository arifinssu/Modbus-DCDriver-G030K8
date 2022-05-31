################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (10.3-2021.10)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../DCMotor/dcmotor.c 

OBJS += \
./DCMotor/dcmotor.o 

C_DEPS += \
./DCMotor/dcmotor.d 


# Each subdirectory must supply rules for building sources it contributes
DCMotor/%.o DCMotor/%.su: ../DCMotor/%.c DCMotor/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m0plus -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32G030xx -c -I../Core/Inc -I../Drivers/STM32G0xx_HAL_Driver/Inc -I../Drivers/STM32G0xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32G0xx/Include -I../Drivers/CMSIS/Include -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfloat-abi=soft -mthumb -o "$@"

clean: clean-DCMotor

clean-DCMotor:
	-$(RM) ./DCMotor/dcmotor.d ./DCMotor/dcmotor.o ./DCMotor/dcmotor.su

.PHONY: clean-DCMotor

