# Modbus-DCDriver-G030K8
> Driver motor dc (DRV8870) controlled with STM32G030K8

### Features
- Customized DCMotor lib
- Filtering with dependancy of [MedianFilter](https://github.com/daPhoosa/MedianFilter), but modified with C syntax
- W25qxx SPI Flash
- Modbus

### Hotfix Focus
- Async DCMotor setup with manipulating HAL_Delay() ticks
- simulating arduino map() function to use with speed timer_period conversion
- Emulating DCMotor_Run() to use with start, stop, acceleration_elapsed_time