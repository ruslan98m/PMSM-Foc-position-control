#ifndef __PID_CONTROLLER__
#define __PID_CONTROLLER__

#include "fmath.h"

typedef struct{
	float Kp;
	float Ki;
	float Kd;
	float I_max;
	float u_max;
	float I_last;
	float err_last;
	float u;
	uint16_t freq;
} pid_t;

typedef struct{
	float Kp;
	float Ki;
	float I_max;
	float u_max;
	float I_last;
	float err_last;
	float u;
	uint16_t freq;
} pi_t;

typedef struct{
	
	float theta;
	uint16_t freq;
	float velocity;
	pi_t controller;
} pll_t;

inline float constrain( float val, float max_val, float min_val);
inline void pid_update( pid_t *pid, float error);
inline void pi_update( pi_t *pi, float error);
inline void pll_update(pll_t *pll, float angle);
	

#endif //__PID_CONTROLLER__