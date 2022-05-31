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
 * @brief Initialize DC Motor
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
    dc.motor_elapsed_time = millisecond;
}

uint32_t DCMotor_GetElapsedTime()
{
    return dc.motor_elapsed_time;
}

uint32_t DCMotor_GetAcceleration()
{
    return dc.motor_acceleration;
}

void DCMotor_HoldSpeed(int speed)
{
    dc.motor_hold_speed = speed;
}

void DCMotor_SetSpeed(int speed)
{
    if(speed != 0) 
    {
        dc.motor_hold_speed = dc.motor_set_speed;
    }

    dc.motor_set_speed = map(speed, 0, 100, 0, Timer_GetMaxPeriod());
}

void DCMotor_SetDirection(uint8_t dir)
{
    dc.motor_direction_flag = dir;
}

uint8_t DCMotor_GetDirectionValue()
{
    return dc.motor_direction_flag;
}

void DCMotor_GetDirection()
{
    if(dc.motor_direction_flag == 1) dc.motor_direction = clockwise;
    else dc.motor_direction = counterclockwise;
}

void DCMotor_Start(uint8_t status)
{
    dc.motor_is_running = status;
}

uint8_t DCMotor_IsRunning()
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

    if(!dc.motor_is_running)
    {
        DCMotor_Stop();
    }

    __update_current_filter();

    // if(dc.motor_current_limit == 1)
    // {
    //     DCMotor_Stop();
    //     dc.motor_current_limit = DCMotor_CalibrateCurrent(30, 50);
    // }

    // if(dc.motor_current_limit != 0 && (dc.motor_current_limit < DCMotor_GetFilteredCurrent()))
    // {
    //     DCMotor_Brake();
    // }

    if(DCMotor_GetCurrentSpeed() == 0)
    {
        DCMotor_GetDirection();
        if(dc.motor_hold_speed != 0) dc.motor_set_speed = dc.motor_hold_speed;
    }

    if((HAL_GetTick() - dc.motor_ticks) > dc.motor_acceleration)
    {
        dc.motor_ticks = HAL_GetTick();
        dc.motor_run();
    }
}


void DCMotor_Stop()
{
    DCMotor_HoldSpeed(DCMotor_GetCurrentSpeed());
    DCMotor_SetSpeed(0);
    if(DCMotor_GetCurrentSpeed() == 0)
    {
        DCMotor_Reset();
    }
    dc.motor_run = &__slower;
}

void DCMotor_Reset()
{
    dc.motor_current_speed = 0;
    dc.motor_hold_speed = 0;
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
        DCMotor_GetDirection();
        __HAL_TIM_SET_COMPARE(&motor_tim, dc.motor_direction, 100);
        HAL_Delay(1);
        for (int j = 0; j <= samples; j++)
        {
            DCMotor_GetSensor(0, time_in_millis);
            HAL_ADC_Stop(&sensor_adc);
            MedianFilter_In(dc.motor_current_filter, HAL_ADC_GetValue(&sensor_adc));
        }
        DCMotor_Brake();
    }

    DCMotor_SetElapsedTime(500);
}

int DCMotor_GetMaxCurrentFilter()
{
    return MedianFilter_GetMax(dc.motor_current_filter);
}

int DCMotor_GetCurrentValue()
{
    DCMotor_GetSensor(0, 1);
    HAL_ADC_Stop(&sensor_adc);
    return (((HAL_ADC_GetValue(&sensor_adc) / 4096.0) * 10.5) * 5.0) * 100;
}

int DCMotor_GetFilteredCurrent()
{
    return MedianFilter_Out(dc.motor_current_filter);
}

int DCMotor_GetVoltageValue()
{
    DCMotor_GetSensor(1, 1);
    HAL_ADC_Stop(&sensor_adc);
    return ((HAL_ADC_GetValue(&sensor_adc) / 4096.0) * 43.0) * 100;
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

int DCMotor_GetCurrentSpeed()
{
    return dc.motor_current_speed;
}