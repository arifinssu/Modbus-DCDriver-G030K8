################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (10.3-2021.10)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../MedianFilter/MedianFilter.c 

OBJS += \
./MedianFilter/MedianFilter.o 

C_DEPS += \
./MedianFilter/MedianFilter.d 


# Each subdirectory must supply rules for building sources it contributes
MedianFilter/%.o MedianFilter/%.su: ../MedianFilter/%.c MedianFilter/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m0plus -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32G030xx -c -I../Core/Inc -I../Drivers/STM32G0xx_HAL_Driver/Inc -I../Drivers/STM32G0xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32G0xx/Include -I../Drivers/CMSIS/Include -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfloat-abi=soft -mthumb -o "$@"

clean: clean-MedianFilter

clean-MedianFilter:
	-$(RM) ./MedianFilter/MedianFilter.d ./MedianFilter/MedianFilter.o ./MedianFilter/MedianFilter.su

.PHONY: clean-MedianFilter

