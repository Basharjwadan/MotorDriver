#include "Motor.h"


Motor::Motor(int dir, int PWM)
{
    m_Dir = dir;
    m_PWM = PWM;

    m_EncCounts = 0;
    m_DesiredRate = 0;
    m_CurrentRate = 0;
    m_PrevEncCount = 0;

    m_LeftPID = new PID();
    m_LeftPID->SetTime(0.033);
    m_LeftPID->SetTau(0.5);
    m_LeftPID->SetLimits(-255.0, 255.0);
    m_LeftPID->SetIntegralLimits(-200.0, 200.0);
    SetPID(0.2f, 6.6f, 0.02f);

    
    pinMode(m_Dir, OUTPUT);
    pinMode(m_PWM, OUTPUT);

}

void Motor::SetMotorSpeed(int speed)
{
    if(speed > 0)
        digitalWrite(m_Dir, HIGH);
    else
        digitalWrite(m_Dir, LOW);

    analogWrite(m_PWM, speed);
}

void Motor::Move()
{
  m_CurrentRate  = GetEncCounts() - m_PrevEncCount;
  Serial.print("Current Rate: ");
  Serial.print(m_CurrentRate);  
  Serial.print("\t\tDesired Rate: ");
  Serial.println(m_DesiredRate);  
  
  MoveAtRate(m_DesiredRate);
  m_PrevEncCount = GetEncCounts();
}

void Motor::SetPID(float kp, float ki, float kd)
{
  m_LeftPID->SetGains(kp, ki, kd);
}

void Motor::MoveAtRate(int rate)
{
  // calc PWD using PID
  int pwm = m_LeftPID->Update((float)m_DesiredRate, (float)m_CurrentRate);
  Serial.println(pwm);
  //send it to the driver
  SetMotorSpeed(pwm);
}
