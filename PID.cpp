#include "PID.h"
#include <Arduino.h>



PID::PID()
{
  m_Config = new PIDconfig;
  /* Clear controller variables */
  m_Config->integrator = 0.0f;
	m_Config->prevError  = 0.0f;

	m_Config->differentiator  = 0.0f;
	m_Config->prevMeasurement = 0.0f;

	m_Config->out = 0.0f;
}

int PID::Update(float setpoint, float measurement)
{
    PIDconfig* pid = m_Config;
    /*
	* Error signal
	*/
    float error = setpoint - measurement;


	/*
	* Proportional
	*/
    float proportional = pid->Kp * error;


	/*
	* Integral
	*/
    pid->integrator = pid->integrator + 0.5f * pid->Ki * pid->T * (error + pid->prevError);

    /* Anti-wind-up via integrator clamping */
    if (pid->integrator > pid->limMaxInt) {

        pid->integrator = pid->limMaxInt;

    } else if (pid->integrator < pid->limMinInt) {

        pid->integrator = pid->limMinInt;

    }

	/*
	* Derivative (band-limited differentiator)
	*/
		
    pid->differentiator = -(2.0f * pid->Kd * (error + pid->prevError)  /* Note: derivative on measurement, therefore minus sign in front of equation! */
                        + (2.0f * pid->tau - pid->T) * pid->differentiator)
                        / (2.0f * pid->tau + pid->T);


	/*
	* Compute output and apply limits
	*/

    Serial.print("tau: ");
    Serial.print(pid->tau);
    Serial.print(" \t\tP: ");
    Serial.print(proportional);  
    Serial.print("\t\t I: ");
    Serial.print( pid->integrator);
    Serial.print("\t\t D ");  
    Serial.println( pid->differentiator); 
    
    pid->out = proportional + pid->integrator + pid->differentiator;

    if (pid->out > pid->limMax) {

        pid->out = pid->limMax;

    } else if (pid->out < pid->limMin) {

        pid->out = pid->limMin;

    }

	/* Store error and measurement for later use */
    pid->prevError       = error;
    pid->prevMeasurement = measurement;

	/* Return controller output */
    return (int)pid->out;
}
