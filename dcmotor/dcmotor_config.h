#ifndef DCMotor_Config_h
#define DCMotor_Config_h

#define motor_tim           htim3
#define motor_plus_pin      TIM_CHANNEL_2
#define motor_minus_pin     TIM_CHANNEL_1
#define motor_sd_pin        TIM_CHANNEL_4

#define adc_handle          hadc1
#define current_sensor_pin  ADC_CHANNEL_0
#define voltage_sensor_pin  ADC_CHANNEL_10
#define adc_rank            ADC_REGULAR_RANK_1
#define adc_sampling        ADC_SAMPLINGTIME_COMMON_1

#endif