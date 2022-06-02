#ifndef DCMotor_h
#define DCMotor_h

#ifdef __cplusplus
extern "C"
{
#endif

    #include "main.h"
    #include "tim.h"
    #include "adc.h"
    #include "DCMotor_Config.h"
    #include "../MedianFilter/MedianFilter.h"

    typedef struct 
    {
        uint8_t         motor_is_running;
        int             motor_current_speed;
        int             motor_hold_speed;
        uint32_t        motor_acceleration;
        uint32_t        motor_elapsed_time;
        uint32_t        motor_direction;
        uint8_t         motor_direction_flag;
        MedianFilter*   motor_current_filter;
        int             motor_voltage_limit;
        int             motor_current_limit;
        int             motor_set_speed;
        uint32_t        motor_ticks;
        void            (*motor_run)();
    } DCMotor_t;

    extern DCMotor_t dc;
    void DCMotor_Start();
    uint8_t DCMotor_Status();
    void DCMotor_GetDirection();
    void DCMotor_Init();
    void DCMotor_SetElapsedTime(uint32_t millisecond);
    uint32_t DCMotor_GetAcceleration();
    void DCMotor_HoldSpeed(int speed);
    void DCMotor_SetSpeed(int speed);
    void DCMotor_SetDirection(uint8_t dir);
    void DCMotor_Update();
    void DCMotor_Stop();
    void DCMotor_Reset();
    void DCMotor_Brake();
    void DCMotor_GetSensor(uint8_t channel, int time_in_millis);
    int DCMotor_GetMaxCurrentFilter();
    int DCMotor_CalibrateCurrent(uint8_t total_sample, int time_in_millis);
    int DCMotor_GetCurrentValue();
    int DCMotor_GetVoltageValue();
    void DCMotor_SetCurrentLimit(int value);
    void DCMotor_SetVoltageLimit(int value);
    int DCMotor_GetCurrentLimit();
    int DCMotor_GetVoltageLimit();
    int DCMotor_GetCurrentSpeed();
    uint8_t DCMotor_GetDirectionValue();
    int DCMotor_GetFilteredCurrent();
    uint32_t DCMotor_GetElapsedTime();

#ifdef __cplusplus
}
#endif

#endif