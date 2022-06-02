# Modbus-DCDriver-G030K8
> Driver motor dc (DRV8870) controlled with STM32G030K8

### Features
- Customized DCMotor lib
- Filtering with dependancy of [MedianFilter](https://github.com/daPhoosa/MedianFilter), but modified with C syntax
- W25qxx SPI Flash
- Modbus

### Fixed Bug
- Async DCMotor setup
- mapping dutycycle
- DCMotor_Update() only on loop (renamed from DCMotor_Run function)

### Known Bug (will focus on next Hotfix)
- Calibrating current fatal error with async

### New Hotfix Focus
- Calibrating current with async
- Cleaning DCMotor_Update()
- Refactoring if no new bug found