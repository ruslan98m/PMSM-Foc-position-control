
#include "motor_calibration.h"


float cur_data_arr[2*ARR_LEN];

void motor_calibrate(servo_t * motor)
{
	
	HAL_Delay(500);
	
	
	for(uint16_t i=0; i<ARR_LEN; i++)
	{
		motor->target_angle += ANG_DIF;
		HAL_Delay(DELAY_TIME);
		cur_data_arr[i] = motor->rotor_vector.q;	
	}
	HAL_Delay(500);
	for(uint16_t i=ARR_LEN; i<2*ARR_LEN; i++)
	{
		motor->target_angle -= ANG_DIF;
		HAL_Delay(DELAY_TIME);
		cur_data_arr[i] = motor->rotor_vector.q;	
	}
//	HAL_FLASH_Unlock();
//	
//	HAL_FLASH_Lock();
}