#include "DCMotor.h"
DCMotor_t dc;

long map(long x, long in_min, long in_max, long out_min, long out_max)
{
    return (x - in_min) * (out_max - out_min + 1) / (in_max - in_min + 1) + out_min;
}

uint32_t Timer_GetMaxPeriod()
{
    return motor_tim.Init.Period;
}

/**
 * @brief Initialize DCMotor instance.
 * 
 */
void DCMotor_Init()
{
    dc.motor_is_running = 0;
    dc.motor_direction_flag = 1;
    dc.motor_current_speed = 0;
    dc.motor_set_speed = 0;
    dc.motor_elapsed_time = 500;
    dc.motor_current_filter = New_MedianFilter(30, 0);
    
    HAL_TIM_PWM_Start(&motor_tim, motor_plus_pin);
    HAL_TIM_PWM_Start(&motor_tim, motor_minus_pin);
    HAL_TIM_PWM_Start(&motor_tim, motor_sd_pin);
    __HAL_TIM_SET_COMPARE(&motor_tim, motor_plus_pin, 0);
    __HAL_TIM_SET_COMPARE(&motor_tim, motor_minus_pin, 0);
    __HAL_TIM_SET_COMPARE(&motor_tim, motor_sd_pin, Timer_GetMaxPeriod());

    DCMotor_GetDirection();
}

/**
 * @brief Set elapsed time to get Acceleration.
 * 
 * @param time in millisecond.
 */
void DCMotor_SetElapsedTime(uint32_t millisecond)
{
    dc.motor_elapsed_time = millisecond;
}

/**
 * @brief Get elapsed time setting.
 * 
 * @return uint32_t - time in millisecond.
 */
uint32_t DCMotor_GetElapsedTime()
{
    return dc.motor_elapsed_time;
}

/**
 * @brief Get Acceleration calculated from elapsed time setting.
 * 
 * @return uint32_t - acceleration.
 */
uint32_t DCMotor_GetAcceleration()
{
    return dc.motor_acceleration;
}

/**
 * @brief Set hold speed to use with direction and speed set.
 * 
 * @param speed
 */
void DCMotor_HoldSpeed(int speed)
{
    dc.motor_hold_speed = speed;
}

/**
 * @brief Set motor speed duty cycle.
 * 
 * @param speed from 0 to 100%
 */
void DCMotor_SetSpeed(int speed)
{
    dc.motor_set_speed = map(speed, 0, 100, 0, Timer_GetMaxPeriod());
}

/**
 * @brief Set motor direction.
 * 
 * @param direction 0 for counterclockwise, 1 for clockwise.
 */
void DCMotor_SetDirection(uint8_t dir)
{
    dc.motor_direction_flag = dir;
}

uint8_t DCMotor_GetDirectionValue()
{
    return dc.motor_direction_flag;
}

uint32_t __clockwise()
{
    return motor_plus_pin;
}

uint32_t __counterclockwise()
{
    return motor_minus_pin;
}

void DCMotor_GetDirection()
{
    if(dc.motor_direction_flag == 1) dc.motor_direction = __clockwise();
    else dc.motor_direction = __counterclockwise();
}

uint8_t DCMotor_Status()
{
    return dc.motor_is_running;
}

void __update_current_filter()
{
    MedianFilter_In(dc.motor_current_filter, DCMotor_GetCurrentValue());
}

void __faster()
{
    if(dc.motor_current_speed <= dc.motor_set_speed) 
    {
        dc.motor_current_speed += 1;
        __HAL_TIM_SET_COMPARE(&motor_tim, dc.motor_direction, dc.motor_current_speed);
    }
    else return;
}

void __slower()
{
    if(dc.motor_current_speed > dc.motor_set_speed) 
    {
        dc.motor_current_speed -= 1;
        __HAL_TIM_SET_COMPARE(&motor_tim, dc.motor_direction, dc.motor_current_speed);
    }
    else return;
}

void DCMotor_Update()
{
    __update_current_filter();

    if(DCMotor_GetCurrentSpeed() == 0)
    {
        DCMotor_GetDirection();
        if(dc.motor_hold_speed != 0)
        {
            dc.motor_set_speed = dc.motor_hold_speed;
        }
        if(dc.motor_is_running == 0)
        {
            return;
        }
    }
    else
    {
        if(dc.motor_is_running == 0)
        {
            DCMotor_SetSpeed(0);
        }
    }

    if(dc.motor_current_limit != 0 && (dc.motor_current_limit < DCMotor_GetFilteredCurrent()))
    {
        DCMotor_Brake();
    }

    if(dc.motor_current_speed < dc.motor_set_speed)
    {
        dc.motor_acceleration = DCMotor_GetElapsedTime() / dc.motor_set_speed;
        dc.motor_run = &__faster;
    }
    else
    {
        dc.motor_acceleration = DCMotor_GetElapsedTime() / dc.motor_hold_speed;
        dc.motor_run = &__slower;
    }

    if((HAL_GetTick() - dc.motor_ticks) > dc.motor_acceleration)
    {
        dc.motor_ticks = HAL_GetTick();
        dc.motor_run();
    }
}

void DCMotor_Start()
{
    dc.motor_is_running = 1;
}

void DCMotor_Stop()
{
    dc.motor_is_running = 0;
}

void DCMotor_Reset()
{
    dc.motor_is_running = 0;
    dc.motor_current_speed = 0;
    dc.motor_set_speed = 0;
    dc.motor_hold_speed = 0;
    dc.motor_elapsed_time = 500;
    __HAL_TIM_SET_COMPARE(&motor_tim, motor_plus_pin, 0);
    __HAL_TIM_SET_COMPARE(&motor_tim, motor_minus_pin, 0);
}

/**
 * @brief Motor is braking.
 * 
 */
void DCMotor_Brake()
{
    __HAL_TIM_SET_COMPARE(&motor_tim, motor_plus_pin, Timer_GetMaxPeriod());
    __HAL_TIM_SET_COMPARE(&motor_tim, motor_minus_pin, Timer_GetMaxPeriod());
    __HAL_TIM_SET_COMPARE(&motor_tim, motor_sd_pin, Timer_GetMaxPeriod());
    DCMotor_Reset();
}

void DCMotor_GetSensor(uint8_t channel, int time_in_millis)
{
    ADC_ChannelConfTypeDef adc = {0};
    if (channel == 0)
        adc.Channel = current_sensor_pin;
    if (channel == 1)
        adc.Channel = voltage_sensor_pin;

    adc.Rank = adc_rank;
    adc.SamplingTime = adc_sampling;
    if (HAL_ADC_ConfigChannel(&adc_handle, &adc) != HAL_OK)
        Error_Handler();

    HAL_ADC_Start(&adc_handle);
    HAL_ADC_PollForConversion(&adc_handle, time_in_millis);
}

int DCMotor_CalibrateCurrent(uint8_t total_sample, int time_in_millis)
{
    uint8_t samples = total_sample / 2;
    DCMotor_SetElapsedTime(1);

    for(uint8_t i = 0; i < 2; i++)
    {
        DCMotor_SetDirection(i);
        DCMotor_GetDirection();
        __HAL_TIM_SET_COMPARE(&motor_tim, dc.motor_direction, Timer_GetMaxPeriod());
        HAL_Delay(time_in_millis);
        for (uint8_t j = 0; j <= samples; j++)
        {
            DCMotor_GetSensor(0, time_in_millis);
            HAL_ADC_Stop(&adc_handle);
            MedianFilter_In(dc.motor_current_filter, HAL_ADC_GetValue(&adc_handle));
            HAL_Delay(time_in_millis / 10);
        }
        DCMotor_Reset();
    }

    DCMotor_SetElapsedTime(500);
    return DCMotor_GetMaxCurrentFilter();
}

int DCMotor_GetMaxCurrentFilter()
{
    return MedianFilter_GetMax(dc.motor_current_filter);
}

int DCMotor_GetCurrentValue()
{
    DCMotor_GetSensor(0, 1);
    HAL_ADC_Stop(&adc_handle);
    return (((HAL_ADC_GetValue(&adc_handle) / 4096.0) * 10.5) * 5.0) * 100;
}

int DCMotor_GetFilteredCurrent()
{
    return MedianFilter_Out(dc.motor_current_filter);
}

int DCMotor_GetVoltageValue()
{
    DCMotor_GetSensor(1, 1);
    HAL_ADC_Stop(&adc_handle);
    return ((HAL_ADC_GetValue(&adc_handle) / 4096.0) * 43.0) * 100;
}

int DCMotor_GetResistanceValue()
{
    int i = DCMotor_GetFilteredCurrent();
    int v = DCMotor_GetVoltageValue();
    return (v / i) * 100;
}

void DCMotor_SetCurrentLimit(int value)
{
    dc.motor_current_limit = value;
}

void DCMotor_SetVoltageLimit(int value)
{
    dc.motor_voltage_limit = value;
}

int DCMotor_GetCurrentLimit()
{
    return dc.motor_current_limit;
}

int DCMotor_GetVoltageLimit()
{
    return dc.motor_voltage_limit;
}

int DCMotor_GetCurrentSpeed()
{
    return dc.motor_current_speed;
}