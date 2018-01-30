/*
 * JointsAndSensors.cpp
 *
 *  Created on: 2016年4月13日
 *      Author: carrol
 */

#include "JointsAndSensors.h"

#include <iostream>
#include <cmath>

static const float DEG_OVER_RAD = 180 / M_PI;
static const float RAD_OVER_DEG = M_PI / 180;
float DEG2RAD(float x)
{
	return ((x) * RAD_OVER_DEG);
}
float RAD2DEG(float x)
{
	return ((x) * DEG_OVER_RAD);
}



