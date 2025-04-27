#pragma once
#ifndef __MATHS_H
#define __MATHS_H

#include <stdint.h>


#ifndef sq
#define sq(x) ((x)*(x))
#endif

#define ABS(x) 		(((x) < 0) ? (-x) : (x))

float applyDeadbandf(float value, float deadband);
float constrainf(float amt, float low, float high);
#endif
