#ifndef MATH_MACRO_H__
#define MATH_MACRO_H__

#include "math_const.h"

#ifndef MAX
#define MAX(a, b) (((a) > (b)) ? (a) : (b))
#endif
#ifndef MIN
#define MIN(a, b) (((a) < (b)) ? (a) : (b))
#endif

#define SQ(a)			 	((a) * (a))
#define ABS(a) 				( ((a) < 0) ? -(a) : (a) )
#define DBAND(a, l, h) 		( (((a) <= (h)) && ((a) >= (l))) ? 0 : (a) )
#define CLIP(a, l, h) 		( MAX((MIN((a), (h))), (l)) )
#define CLIPL(a, l) 		( ((a) < (l)) ? (l) : (a))
#define CLIPH(a, h) 		( ((a) > (h)) ? (h) : (a))
#define CLIPS(a, r) 		( MAX((MIN((a), (r))), (-r)) )
#define SIGN(a) 			( ((a) > 0) ? 1 : (((a) == 0) ? 0 : -1) )
#define DIRECTION(a) 		( ((a) >= 0) ? 1 : -1 )
#define INRANGE(a, l, h) 	( (((a) > (l)) && ((a) <= (h))) || ((a) == (l)) )
#define ROUND(a) 			( ABS((a) - ((int)(a))) >= 0.5 ? ((int)(a)) + SIGN(a) : (int)(a) )
#define DECAY(a, l, h)     	CLIP((((a) - (l)) / ((h) - (l))), 0.0, 1.0)
#define INV_DECAY(a, l, h) 	CLIP((((h) - (a)) / ((h) - (l))), 0.0, 1.0)
#define DEG_TO_RAD(a)		((a) * PI / 180.0f)
#define RAD_TO_DEG(a)		((a) * 180.0f / PI)
#define LOGIC_XOR(a, b)		(((a) || (b)) && !((a) && (b)))
#define MOD(a, m)			((a) % (m) + ((a < 0) ? (m) : 0))

//MIX result = (1-k) * a + k * b
#define MIX(a, b, k)		( (k) * ((b) - (a)) + (a) )



#endif
