#include "Motor.h"


void MotorSetSpeed(Motor* motor, int speed)
{
  if(speed > 0)
  {
        digitalWrite(motor->m_Dir, HIGH);
  }
    else
    {
        digitalWrite(motor->m_Dir, LOW);
        speed = -1 * speed;
    }

    analogWrite(motor->m_Pwm, speed);
}

void MotorInstantStop(const Motor& motor)
{
  analogWrite(motor.m_Pwm, 0);
}

void MotorMove(Motor* motor)
{
   int pwm = PIDController_Update(motor->m_Pid, motor->m_DesiredMotorRate, motor->m_CurrentMotorRate);
   MotorSetSpeed(motor, pwm);
}
