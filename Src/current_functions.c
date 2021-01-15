#include "current_functions.h"


inline cur_dq_t clark_transform(cur_abc_t* phase_current)
{
	cur_dq_t cur_space_vector;
	cur_space_vector.q = phase_current->a;
	cur_space_vector.d = (phase_current->b - phase_current->c) * ISQRT3;
	return cur_space_vector;
}

inline cur_dq_t park_transform(cur_dq_t* cur_vect, float theta)
{
	cur_dq_t cur_rotor_vector;
	float sin_theta, cos_theta;
	_sincos_1_(theta, &sin_theta, &cos_theta);
	cur_rotor_vector.q = cur_vect->q * cos_theta - cur_vect->d * sin_theta;
	cur_rotor_vector.d = cur_vect->q * sin_theta + cur_vect->d * cos_theta;
	return cur_rotor_vector;
}

inline cur_dq_t inv_park_transform(cur_dq_t* volt_vect, float theta)
{
	cur_dq_t cur_space_vector;
	float sin_theta, cos_theta;
	_sincos_1_(theta, &sin_theta, &cos_theta);
	cur_space_vector.q = volt_vect->q * cos_theta + volt_vect->d * sin_theta;
	cur_space_vector.d = -volt_vect->q * sin_theta + volt_vect->d * cos_theta;
	return cur_space_vector;
}

inline cur_abc_t inv_clark_transform(cur_dq_t* volt_vect)
{
	cur_abc_t phase_voltage;
	phase_voltage.a = volt_vect->q;
	phase_voltage.b = 0.8660254038f * volt_vect->d - 0.5f * volt_vect->q;
	phase_voltage.c = -0.8660254038f * volt_vect->d - 0.5f * volt_vect->q;
	return phase_voltage;
}