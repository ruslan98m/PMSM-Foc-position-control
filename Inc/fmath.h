#ifndef FMATH_DOT_H_
#define FMATH_DOT_H_

#include <math.h>
#include <stdint.h>
#include "math_macro.h"

#define _ATAN2_ _atan2_
#define _SQRT_ sqrtf
#define _SIN_ sinf
#define _COS_ cosf
#define _SINCOS_ _sincos_1_
//#define _ISQRT_(x) (1.0f / sqrtf(x))
#define _ISQRT_ _isqrt_q3a_
#define _MOD_ _mod_


static inline float _mod_(float v, float m)
{
	v -= m * (int32_t)(v / m);
	return (v < 0) ? v + m : v;
}

static inline float calculate_err(float err){
	if(err>PI)
		return err-2*PI;
	else if(err<-PI)
		return err+2*PI;
	return err;
}
static inline void _sincos_1_(float angle, float *sin, float *cos)
{
	//always wrap input angle to -PI..PI
	while(angle < -M_PI)
		angle += M_PI * 2.0f;

	while(angle > M_PI)
		angle -= M_PI * 2.0f;

	//compute sine
	if(angle < 0.0)
	{
		*sin = 1.27323954f * angle + 0.405284735f * angle * angle; /* angle * ( 1.27323954 + (0.405284735 * angle)) */

		if(*sin < 0.0)
			*sin = 0.225 * (*sin * -*sin - *sin) + *sin; /* (*sin) * ((-0.225 * ( (*sin) + 1 )) + 1) */
		else
			*sin = 0.225 * (*sin * *sin - *sin) + *sin; /* (*sin) * ((0.225 * ( (*sin) - 1))  + 1) */
	}
	else
	{
		*sin = 1.27323954 * angle - 0.405284735 * angle * angle; /* angle * ( 1.27323954 - (0.405284735 * angle)) */

		if(*sin < 0.0)
			*sin = 0.225 * (*sin * -*sin - *sin) + *sin; /* (*sin) * ((-0.225 * ( (*sin) + 1 )) + 1) */
		else
			*sin = 0.225 * (*sin * *sin - *sin) + *sin; /* (*sin) * ((0.225 * ((*sin) - 1))  + 1) */
	}

	// compute cosine: sin(x + PI/2) = cos(x)
	angle += M_PI * 0.5f;

	if(angle > 3.14159256)
		angle -= M_PI * 2.0f;

	if(angle < 0.0)
	{
		*cos = 1.27323954 * angle + 0.405284735 * angle * angle; /* angle * ( 1.27323954 + (0.405284735 * angle)) */

		if(*cos < 0.0)
			*cos = 0.225 * (*cos * -*cos - *cos) + *cos; /* (*cos) * ((-0.225 * ( (*cos) + 1 )) + 1) */
		else
			*cos = 0.225 * (*cos * *cos - *cos) + *cos; /* (*cos) * ((0.225 * ( (*cos)- 1 )) + 1) */
	}
	else
	{
		*cos = 1.27323954 * angle - 0.405284735 * angle * angle; /* angle * ( 1.27323954 - (0.405284735 * angle)) */

		if(*cos < 0.0)
			*cos = 0.225 * (*cos * -*cos - *cos) + *cos;
		else
			*cos = 0.225 * (*cos * *cos - *cos) + *cos;
	}
}


#define FM_2PI ((float)(2.0f * M_PI))   /*!< \brief definition of 2 times Pi (2*3.14159...) */
#define FM_PI ((float)PI)           /*!< \brief definition Pi */
#define FM_PI_2 ((float)(0.5f * M_PI))       /*!< \brief definition Pi/2 (2*3.14159...) */
#define FM_PI_4 ((float)(0.25f * M_PI))       /*!< \brief definition of Pi/4 (2*3.14159...) */


#define F 0.28f

static inline float _atan_(float y,float x, float offset)
{
	float result;
	float y2 = y * y;
	float x2 = x * x;

	if(y2 <= x2)
	{
		/* |y/x| <= 1 */
		result = x * y / (x2 + F * y2);
	}
	else
	{
		union {float f; uint32_t i;} yu, xu;

		xu.f = x;
		yu.f = y;
		if((xu.i ^ yu.i) & 0x80000000)
		{
			/* |y/x| < -1 */
			result = -FM_PI_2 - x * y / (y2 + F * x2);
		}
		else
		{
			/* |y/x| > +1 */
			result = +FM_PI_2 - x * y / (y2 + F * x2);
		}
	}
	return result + offset;
}

static inline float _atan2_(float y,float x)
{
	float result;

	if(x > EPS)
	{
		result = _atan_ (y,x, 0.0f);
	}
	else if(x < -EPS)
	{
		if(y >= 0.0f)
		{
			result = _atan_ (y,x, FM_PI);
		}
		else /* y < 0.0 */
		{
			result = _atan_ (y,x,-FM_PI);
		}
	}
	else /* x == 0.0 */
	{
		if(y > EPS)
		{
			result = FM_PI_2;
		}
		else if(y < -EPS)
		{
			result = -FM_PI_2;
		}
		else /* y == 0.0 */
		{
			result = 0.0f;
		}
	}
	return result;
}

/*
a := min (|x|, |y|) / max (|x|, |y|)
s := a * a
r := ((-0.0464964749 * s + 0.15931422) * s - 0.327622764) * s * a + a
if |y| > |x| then r := 1.57079637 - r
if x < 0 then r := 3.14159274 - r
if y < 0 then r := -r
 *
 */
static inline float _atan2_a(float y,float x)
{
	float d, a, s, r;

	d = MAX(ABS(x), ABS(y));

	if(d < EPS)
	{
		return 0;
	}

	a = MIN(ABS(x), ABS(y)) / MAX(ABS(x), ABS(y));
	s = SQ(a);
	r = ((-0.0464964749f * s + 0.15931422f) * s - 0.327622764f) * s * a + a;

	if(ABS(y) > ABS(x))
	{
		r = 0.5f * PI - r;
	}
	if(x < 0)
	{
		r = PI - r;
	}
	if(y < 0)
	{
		r = -r;
	}
	return r;
}


static inline float _sqrt_(float a)
{
	return sqrtf(a);
}    

static inline float _isqrt_q3a_( float number )
{	
	const float x2 = number * 0.5F;
	const float threehalfs = 1.5F;

	union 
	{
		float f;
		uint32_t i;
	} conv = {number}; 
	conv.i  = 0x5f3759df - ( conv.i >> 1 );
	conv.f  *= ( threehalfs - ( x2 * conv.f * conv.f ) );
	return conv.f;
}


static inline float _sin_(float x)
{
	float result, sign = 1.0f;
	float a, b;
	uint32_t idx;
	extern float _fast_sin_table_ [129][2];

	/* make result positive */
	if (x < 0.0f)
	{
		sign = -sign;
		x = -x;
	}

	/* bring results into range [0;2*pi] */
	int n = x / FM_2PI;
	x -= FM_2PI * n;

	/* exploid symmetries */
	if (x > FM_PI)
	{
		sign = -sign;
		x -= FM_PI;
	}
	if (x > FM_PI_2)
	{
		x = FM_PI - x;
	}

	/* now, we can assume x : [0;pi/2] */
	idx = (int)(x * (128.00f / FM_PI_2));
	a = _fast_sin_table_[idx][0];
	b = _fast_sin_table_[idx][1];
	result = a * x + b;

	/* apply sign */
	result *= sign;

	return result;
}

static inline float _cos_(float x)
{
	return _sin_(x + FM_PI_2);
}



#endif
