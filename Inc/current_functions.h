#ifndef __CURRENT_FUNCTIONS__
#define __CURRENT_FUNCTIONS__

#include "fmath.h"


typedef struct{
	float d;
	float q;
} cur_dq_t;

typedef struct{
	float a;
	float b;
	float c;
} cur_abc_t;

cur_dq_t clark_transform(cur_abc_t *phase_current);
cur_dq_t park_transform(cur_dq_t *cur_vect, float theta);	
cur_dq_t inv_park_transform(cur_dq_t *volt_vect, float theta);	
cur_abc_t inv_clark_transform(cur_dq_t *volt_vect);	

#endif //CURRENT_FUNCTIONS