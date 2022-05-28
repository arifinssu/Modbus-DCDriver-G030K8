#include "DC_Motor.h"
DC_Motor_t motor;

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
    motor.motor_speed = 0;
    motor.acceleration = 500;
    motor.current_filter = New_MedianFilter(30, 0);
    
    HAL_TIM_PWM_Start(&motor_tim, clockwise);
    HAL_TIM_PWM_Start(&motor_tim, counterclockwise);
    HAL_TIM_PWM_Start(&motor_tim, sd_pin);
    __HAL_TIM_SET_COMPARE(&motor_tim, clockwise, 0);
    __HAL_TIM_SET_COMPARE(&motor_tim, counterclockwise, 0);
    __HAL_TIM_SET_COMPARE(&motor_tim, sd_pin, Timer_GetMaxPeriod());
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
void DCMotor_SetAcceleration(uint32_t millisecond)
{
    motor.acceleration = millisecond;
}

void DCMotor_SetSpeed(int speed)
{
    speed = map(speed, 0, 100, 0, Timer_GetMaxPeriod());
    motor.motor_old_speed = motor.motor_speed;
    motor.motor_speed = speed;
}

void DCMotor_SetDirection(uint8_t dir)
{
    if (dir == 1) motor.direction = clockwise;
    else motor.direction = counterclockwise;
}

void _run_faster(int _delay)
{
    for(int s = motor.motor_old_speed; s <= motor.motor_speed; ++s)
    {
        __HAL_TIM_SET_COMPARE(&motor_tim, motor.direction, s);
        HAL_Delay(_delay);
    }
}
void _run_slower(int _delay)
{
    for(int s = motor.motor_old_speed; s >= motor.motor_speed; --s)
    {
        __HAL_TIM_SET_COMPARE(&motor_tim, motor.direction, s);
        HAL_Delay(_delay);
    }
}

void DCMotor_Run()
{
    int _delay = motor.acceleration / motor.motor_speed;
    if(motor.motor_old_speed < motor.motor_speed) _run_faster(_delay);
    else _run_slower(_delay);
}

void DCMotor_Stop()
{
    int _delay = motor.acceleration / motor.motor_speed;
    for(int s = motor.motor_speed; s >= 0; s--)
    {
        __HAL_TIM_SET_COMPARE(&motor_tim, motor.direction, s);
        HAL_Delay(_delay);
    }
     __HAL_TIM_SET_COMPARE(&motor_tim, motor.direction, 0);
}

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
    DCMotor_SetAcceleration(1);

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

    DCMotor_SetAcceleration(500);
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
    return v / i;
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

int DCMotor_GetSpeed()
{
    return motor.motor_speed;
}

int DCMotor_GetAcceleration()
{
    return motor.acceleration;
}