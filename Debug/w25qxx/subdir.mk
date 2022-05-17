################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (10.3-2021.10)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../w25qxx/w25qxx.c 

OBJS += \
./w25qxx/w25qxx.o 

C_DEPS += \
./w25qxx/w25qxx.d 


# Each subdirectory must supply rules for building sources it contributes
w25qxx/%.o w25qxx/%.su: ../w25qxx/%.c w25qxx/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m0plus -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32G030xx -c -I../Core/Inc -I../Drivers/STM32G0xx_HAL_Driver/Inc -I../Drivers/STM32G0xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32G0xx/Include -I../Drivers/CMSIS/Include -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfloat-abi=soft -mthumb -o "$@"

clean: clean-w25qxx

clean-w25qxx:
	-$(RM) ./w25qxx/w25qxx.d ./w25qxx/w25qxx.o ./w25qxx/w25qxx.su

.PHONY: clean-w25qxx

