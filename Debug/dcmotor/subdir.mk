################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (10.3-2021.10)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../dcmotor/dcmotor.c \
../dcmotor/dcmotor_config.c 

OBJS += \
./dcmotor/dcmotor.o \
./dcmotor/dcmotor_config.o 

C_DEPS += \
./dcmotor/dcmotor.d \
./dcmotor/dcmotor_config.d 


# Each subdirectory must supply rules for building sources it contributes
dcmotor/%.o dcmotor/%.su: ../dcmotor/%.c dcmotor/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m0plus -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32G030xx -c -I../Core/Inc -I../Drivers/STM32G0xx_HAL_Driver/Inc -I../Drivers/STM32G0xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32G0xx/Include -I../Drivers/CMSIS/Include -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfloat-abi=soft -mthumb -o "$@"

clean: clean-dcmotor

clean-dcmotor:
	-$(RM) ./dcmotor/dcmotor.d ./dcmotor/dcmotor.o ./dcmotor/dcmotor.su ./dcmotor/dcmotor_config.d ./dcmotor/dcmotor_config.o ./dcmotor/dcmotor_config.su

.PHONY: clean-dcmotor

