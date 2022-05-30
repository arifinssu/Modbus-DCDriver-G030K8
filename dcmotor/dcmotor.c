#include "DCMotor.h"
DC_Motor_t motor;

#define __slower -=1
#define __faster +=1

long map(long x, long in_min, long in_max, long out_min, long out_max)
{
    return (x - in_min) * (out_max - out_min + 1) / (in_max - in_min + 1) + out_min;
}

uint32_t Timer_GetMaxPeriod()
{
    return motor_tim.Init.Period;
}

/**
 * @brief Initialize DC Motor
 * 
 */
void DCMotor_Init()
{
    motor.is_running = 0;
    motor.direction_flag = 1;
    motor.motor_speed = 0;
    motor.set_speed = 0;
    motor.elapsed_time = 500;
    motor.current_filter = New_MedianFilter(30, 0);
    
    HAL_TIM_PWM_Start(&motor_tim, clockwise);
    HAL_TIM_PWM_Start(&motor_tim, counterclockwise);
    HAL_TIM_PWM_Start(&motor_tim, sd_pin);
    __HAL_TIM_SET_COMPARE(&motor_tim, clockwise, 0);
    __HAL_TIM_SET_COMPARE(&motor_tim, counterclockwise, 0);
    __HAL_TIM_SET_COMPARE(&motor_tim, sd_pin, Timer_GetMaxPeriod());

    DCMotor_GetDirection();
}

/**
 * @brief Deinitialize DC Motor
 * 
 */
void DCMotor_Deinit()
{
    __HAL_TIM_SET_COMPARE(&motor_tim, clockwise, 0);
    __HAL_TIM_SET_COMPARE(&motor_tim, counterclockwise, 0);
    __HAL_TIM_SET_COMPARE(&motor_tim, sd_pin, 0);
    HAL_TIM_PWM_Stop(&motor_tim, clockwise);
    HAL_TIM_PWM_Stop(&motor_tim, counterclockwise);
    HAL_TIM_PWM_Stop(&motor_tim, sd_pin);
}

/**
 * @brief Set Acceleration time
 * 
 * @param millisecond
 */
void DCMotor_SetElapsedTime(uint32_t millisecond)
{
    motor.elapsed_time = millisecond;
}

uint32_t DCMotor_GetAcceleration()
{
    return motor.elapsed_time / motor.set_speed;
}

void DCMotor_SetSpeed(int speed)
{
    speed = map(speed, 0, 100, 0, Timer_GetMaxPeriod());
    motor.motor_old_speed = motor.motor_speed;
    // motor.motor_speed = speed;
    motor.set_speed = speed;
}

void DCMotor_SetDirection(uint8_t dir)
{
    // if (dir == 1) motor.direction = clockwise;
    // else motor.direction = counterclockwise;

    motor.direction_flag = dir; 
}

void DCMotor_Start(uint8_t status)
{
    motor.is_running = status;
}

uint8_t DCMotor_IsRunning()
{
    return motor.is_running;
}

void DCMotor_GetDirection()
{
    // return motor.direction;
    // return motor.direction_flag;

    if (motor.direction_flag == 1) motor.direction = clockwise;
    else motor.direction = counterclockwise;
}

// void DCMotor_Direction()
// {
//     if (DCMotor_GetDirection() == 1) motor.direction = clockwise;
//     else motor.direction = counterclockwise;
// }

void __run_faster()
{
    if (motor.motor_speed <= motor.set_speed) 
    {
        motor.motor_speed += 1;
        __HAL_TIM_SET_COMPARE(&motor_tim, motor.direction, motor.motor_speed);
    }
    else return;
}

void __run_slower()
{
    if (motor.motor_speed > motor.set_speed) 
    {
        motor.motor_speed -= 1;
        __HAL_TIM_SET_COMPARE(&motor_tim, motor.direction, motor.motor_speed);
    }
    else return;
}

void DCMotor_Run()
{
    int _accel;
    if(motor.motor_speed < motor.set_speed)
    {
        _accel = motor.elapsed_time / motor.set_speed;
        motor.__running = &__run_faster;
    }
    else
    {
        _accel = motor.elapsed_time / motor.motor_speed;
        motor.__running = &__run_slower;
    }

    if(!motor.is_running) DCMotor_Stop();

    DCMotor_Update(_accel);
}

void DCMotor_Update(int _accel)
{
    if((HAL_GetTick() - motor.ticks) > _accel)
    {
        motor.ticks = HAL_GetTick();
        motor.__running();
    }
}

// void DCMotor_Stop()
// {
//     DCMotor_SetSpeed(0);
//     if((HAL_GetTick() - motor.ticks) > (motor.elapsed_time / motor.motor_speed))
//     {
//         motor.ticks = HAL_GetTick();
//         motor.__running();
//     }
// }

void DCMotor_Stop()
{
    DCMotor_SetSpeed(0);
    if(DCMotor_GetCurrentSpeed() == 0)
    {
        DCMotor_Reset();
    }
    motor.__running = &__run_slower;
}

// void DCMotor_Stop()
// {
//     int _delay = motor.elapsed_time / motor.motor_speed;
//     for(int s = motor.motor_speed; s >= 0; s--)
//     {
//         __HAL_TIM_SET_COMPARE(&motor_tim, motor.direction, s);
//         HAL_Delay(_delay);
//     }
//      __HAL_TIM_SET_COMPARE(&motor_tim, motor.direction, 0);
// }

void DCMotor_Jogg(uint8_t dir, int speed, int time_in_millis)
{
    DCMotor_SetDirection(dir);
    DCMotor_SetSpeed(speed);
    __HAL_TIM_SET_COMPARE(&motor_tim, motor.direction, motor.motor_speed);
    HAL_Delay(time_in_millis);
    __HAL_TIM_SET_COMPARE(&motor_tim, motor.direction, 0);
}

void DCMotor_Reset()
{
    motor.motor_speed = 0;
    motor.motor_old_speed = 0;
    __HAL_TIM_SET_COMPARE(&motor_tim, clockwise, 0);
    __HAL_TIM_SET_COMPARE(&motor_tim, counterclockwise, 0);
}

/**
 * @brief Motor is braking.
 * 
 */
void DCMotor_Brake()
{
    __HAL_TIM_SET_COMPARE(&motor_tim, clockwise, Timer_GetMaxPeriod());
    __HAL_TIM_SET_COMPARE(&motor_tim, counterclockwise, Timer_GetMaxPeriod());
    __HAL_TIM_SET_COMPARE(&motor_tim, sd_pin, Timer_GetMaxPeriod());
    DCMotor_Reset();
}

void DCMotor_GetSensor(uint8_t channel, int time_in_millis)
{
    ADC_ChannelConfTypeDef sConfig = {0};
    if (channel == 0)
        sConfig.Channel = current_sensor_pin;
    if (channel == 1)
        sConfig.Channel = voltage_sensor_pin;

    sConfig.Rank = adc_rank;
    sConfig.SamplingTime = adc_sampling;
    if (HAL_ADC_ConfigChannel(&sensor_adc, &sConfig) != HAL_OK)
        Error_Handler();

    HAL_ADC_Start(&sensor_adc);
    HAL_ADC_PollForConversion(&sensor_adc, time_in_millis);
}

void DCMotor_CalibrateCurrent(int total_sample, int time_in_millis)
{
    int samples = total_sample / 2;
    DCMotor_SetElapsedTime(1);

    for(int i = 0; i < 2; i++)
    {
        DCMotor_SetDirection(i);
        DCMotor_SetSpeed(100);
        HAL_Delay(1);
        DCMotor_Run();
        for (int j = 0; j <= samples; j++)
        {
            DCMotor_GetSensor(0, time_in_millis);
            HAL_ADC_Stop(&sensor_adc);
            MedianFilter_In(motor.current_filter, HAL_ADC_GetValue(&sensor_adc));
        }
        DCMotor_Brake();
    }

    DCMotor_SetElapsedTime(500);
}

int DCMotor_GetMaxCurrentFilter()
{
    return MedianFilter_GetMax(motor.current_filter);
}

int DCMotor_GetCurrentValue()
{
    DCMotor_GetSensor(0, 1);
    HAL_ADC_Stop(&sensor_adc);
    return (((HAL_ADC_GetValue(&sensor_adc) / 4096.0) * 10.5) * 5.0) * 100;
}

int DCMotor_GetVoltageValue()
{
    DCMotor_GetSensor(1, 1);
    HAL_ADC_Stop(&sensor_adc);
    return ((HAL_ADC_GetValue(&sensor_adc) / 4096.0) * 43.0) * 100;
}

int DCMotor_GetResistanceValue()
{
    int i = DCMotor_GetCurrentValue();
    int v = DCMotor_GetVoltageValue();
    return (v / i) * 100;
}

void DCMotor_SetCurrentLimit(int value)
{
    motor.current_limit = value;
}

void DCMotor_SetVoltageLimit(int value)
{
    motor.voltage_limit = value;
}

int DCMotor_GetCurrentLimit()
{
    return motor.current_limit;
}

int DCMotor_GetCurrentSpeed()
{
    return motor.motor_speed;
}