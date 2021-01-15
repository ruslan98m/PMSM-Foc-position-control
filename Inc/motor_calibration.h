#ifndef MOTOR_CALIB
#define MOTOR_CALIB

#include "main.h"
#include "current_functions.h"
#include "math.h"

#define ARR_LEN (uint16_t)1024
#define ANG_DIF (float)(2*PI/ARR_LEN)
#define DELAY_TIME 20
	
typedef struct
{
	uint16_t cur_a_1;
	uint16_t cur_a_2;
	uint16_t cur_b_1;
	uint16_t cur_b_2;
	uint16_t volt_1;
	uint16_t volt_2;
	
}adc_res_t;

typedef struct
{
	float theta;
	float velocity;
	float voltage;
	float current;
	float torque;
	float current_theta;
	float angle;
	float target_angle;
	
	cur_abc_t phase_current;
	cur_dq_t space_vector;
	cur_dq_t rotor_vector;
	uint8_t pole_pairs;
	
}servo_t;


void motor_calibrate(servo_t * motor);


#endif //MOTOR_CALIB