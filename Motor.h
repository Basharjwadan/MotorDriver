#ifndef MOTOR_H
#define MOTOR_H
#include <Arduino.h>
#include "PID.h"



class Motor
{
public:
    Motor(int dir, int PWM);
    void SetMotorSpeed(int speed);
    void Move();
    void SetPID(float kp, float ki, float kd);
    inline void SetMotorRate(int rate) { m_DesiredRate = rate; }
    inline int& GetEncCounts() { return m_EncCounts; }
    inline void ResetEncCounts() { m_EncCounts = 0; }

  
private:
    Motor(Motor&) = delete;
    Motor& operator=(Motor&) = delete;
    void MoveAtRate(int rate);
    
private:
    int m_Dir;
    int m_PWM;
    int m_EncCounts;
    int m_PrevEncCount;
    int m_DesiredRate;
    int m_CurrentRate;
    PID* m_LeftPID;

};
#endif
