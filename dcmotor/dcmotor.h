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
        uint8_t is_running;
        int                 motor_speed;
        int                 motor_old_speed;
        uint32_t            acceleration;
        uint32_t            elapsed_time;
        uint32_t            direction;
        uint8_t direction_flag;
        MedianFilter*       current_filter;
        int                 voltage_limit;
        int                 current_limit;
        int set_speed;
        uint32_t ticks;
        void (*__running)();

    } DC_Motor_t;

    extern DC_Motor_t motor;
    void DCMotor_Start(uint8_t status);
    uint8_t DCMotor_IsRunning();
    void DCMotor_GetDirection();
    void DCMotor_Init();
    void DCMotor_Deinit();
    void DCMotor_SetElapsedTime(uint32_t millisecond);
    uint32_t DCMotor_GetAcceleration();
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
    int DCMotor_GetCurrentSpeed();

#ifdef __cplusplus
}
#endif

#endif