#include "PID_controller.h"

inline static float constrain( float val, float max_val, float min_val)
{
	if(val > max_val)
		val = max_val;
	if(val < min_val)
		val = min_val;
	return val;
}
	


void pid_update( pid_t *pid, float error )
{
	float P = pid->Kp*error;	
	float I = pid->I_last + pid->Ki *  error;
	I = constrain(I, pid->I_max, -pid->I_max);	
	float D = pid->Kd * (error - pid->err_last)*pid->freq;
	pid->err_last = error;
	pid->I_last = I;
	
	pid->u = constrain(P+I+D, pid->u_max, -pid->u_max);
}

void pi_update( pi_t *pi, float error )
{
	float P = pi->Kp*error;
	float I = pi->I_last + pi->Ki * error/(float)pi->freq;
	I = constrain(I, pi->I_max, -pi->I_max);
	pi->I_last = I;
	
	pi->u = constrain(P+I, pi->u_max, -pi->u_max);
}

void pll_update(pll_t *pll, float angle)
{
	float theta_con = fmodf(pll->theta, 2*PI); 
	theta_con += theta_con < 0 ? 2*PI : 0;
	
	pi_update(&pll->controller, calculate_err(angle - theta_con));
	pll->velocity = pll->controller.u;
	pll->theta += pll->velocity/pll->freq;
}