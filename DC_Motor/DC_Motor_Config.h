#ifndef DC_Motor_Config_h
#define DC_Motor_Config_h

#define motor_tim           htim3
#define clockwise           TIM_CHANNEL_2
#define counterclockwise    TIM_CHANNEL_1
#define sd_pin              TIM_CHANNEL_4

#define sensor_adc          hadc1
#define current_sensor_pin  ADC_CHANNEL_0
#define voltage_sensor_pin  ADC_CHANNEL_10
#define adc_rank            ADC_REGULAR_RANK_1
#define adc_sampling        ADC_SAMPLINGTIME_COMMON_1

#endif