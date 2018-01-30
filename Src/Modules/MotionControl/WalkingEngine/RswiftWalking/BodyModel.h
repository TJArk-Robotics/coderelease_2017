/*
 * Odometry.hpp
 *
 *  Created on: 2016年4月10日
 *      Author: carrol
 */

#pragma once

#include <iostream>
#include <vector>
#include "JointsAndSensors.h"
#include "Tools/Joints.h"
//#include "SensorValues.hpp"

class BodyModel
{
 public:
  BodyModel();
  void setIsLeftPhase(bool b);
  void updateZMPL(SensorValues &sensors);
  bool isLeftPhase;
  float ZMPL;
  float lastZMPL;

 private:
  float fsLfr, fsLfl, fsLrr, fsLrl;  // Maximum foot sensor reading during latest run
  float fsRfr, fsRfl, fsRrr, fsRrl;  // ... used to scale each foot sensor to read in similar range
};





