


class PID
{
	typedef struct {

	/* Controller gains */
	float Kp;
	float Ki;
	float Kd;

	/* Derivative low-pass filter time constant */
	float tau;

	/* Output limits */
	float limMin;
	float limMax;
	
	/* Integrator limits */
	float limMinInt;
	float limMaxInt;

	/* Sample time (in seconds) */
	float T;

	/* Controller "memory" */
	float integrator;
	float prevError;			/* Required for integrator */
	float differentiator;
	float prevMeasurement;		/* Required for differentiator */

	/* Controller output */
	float out;

	} PIDconfig;


public:
	explicit PID();
	int Update(float setpoint, float measurement);
	inline void SetGains(float kp, float ki, float kd) { m_Config->Kp = kp; m_Config->Ki = ki; m_Config->Kd = kd; }
	inline void SetTime(float time) { m_Config->T = time; }
	inline void SetTau(float tau) { m_Config->tau = tau; }
	inline void SetLimits(float min, float max) { m_Config->limMin = min; m_Config->limMax = max; }
	inline void SetIntegralLimits(float min, float max) { m_Config->limMinInt = min; m_Config->limMaxInt = max; }

private:
	PIDconfig* m_Config;
};
