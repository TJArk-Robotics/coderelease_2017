/*
 * MotionOdometry.hpp
 *
 *  Created on: 2016年4月13日
 *      Author: carrol
 */

#pragma once

#include "JointsAndSensors.h"

struct Odometry {
   float forward;
   float left;
   float turn;

   Odometry(float f = 0.0f, float l = 0.0f, float t = 0.0f)
      : forward(f), left(l), turn(t) {}

   inline void clear() {
      forward = left = turn = 0.0f;
   }
};

class MotionOdometry {
public:
	MotionOdometry();
	Odometry updateOdometry(const SensorValues &sensors, Odometry walkChange);
	void reset();

private :
	bool isEnabled;
	float slipAverage;
	std::vector<Odometry> walkChangeBuffer;
};
