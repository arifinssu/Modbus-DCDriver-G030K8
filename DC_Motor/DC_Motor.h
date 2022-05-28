#ifndef DC_Motor_h
#define DC_Motor_h

#ifdef __cplusplus
extern "C"
{
#endif

    #include "main.h"
    #include "tim.h"
    #include "adc.h"
    #include "DC_Motor_Config.h"
    #include "../MedianFilter/MedianFilter.h"

    typedef struct 
    {
        int                 motor_speed;
        int                 motor_old_speed;
        uint32_t            acceleration;
        uint32_t            direction;
        MedianFilter*       current_filter;
        int                 voltage_limit;
        int                 current_limit;
    } DC_Motor_t;

    extern DC_Motor_t motor;
    void DCMotor_Init();
    void DCMotor_Deinit();
    void DCMotor_SetAcceleration(uint32_t milliseconds);
    void DCMotor_SetSpeed(int speed);
    void DCMotor_SetDirection(uint8_t dir);
    void DCMotor_Run();
    void DCMotor_Stop();
    void DCMotor_Jogg(uint8_t dir, int speed, int time_in_millis);
    void DCMotor_Reset();
    void DCMotor_Brake();
    void DCMotor_GetSensor(uint8_t channel, int time_in_millis);
    void DCMotor_CalibrateCurrent(int total_sample, int time_in_millis);
    int DCMotor_GetMaxCurrentFilter();
    int DCMotor_GetCurrentValue();
    int DCMotor_GetVoltageValue();
    void DCMotor_SetCurrentLimit(int value);
    void DCMotor_SetVoltageLimit(int value);
    int DCMotor_GetCurrentLimit();
    int DCMotor_GetSpeed();
    int DCMotor_GetAcceleration();

#ifdef __cplusplus
}
#endif

#endif