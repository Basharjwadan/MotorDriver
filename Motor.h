#ifndef MOTOR_H
#define MOTOR_H
#include "PID.h"


typedef struct 
{
    int m_Dir;
    int m_Pwm;

    PIDController* m_Pid;
    int m_CurrentMotorRate;
    int m_DesiredMotorRate;

    int m_PrevEncCounts;
} Motor;

void MotorSetSpeed( Motor* motor, int speed);
void MotorMove(Motor* motor);
void MotorInstantStop(const Motor& motor);

#endif
