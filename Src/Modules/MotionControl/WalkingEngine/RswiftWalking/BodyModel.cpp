/*
 * BodyModel.cpp
 *
 *  Created on: 2016年4月13日
 *      Author: carrol
 */

#include "BodyModel.h"

using namespace Sensors;

BodyModel::BodyModel()
{
  isLeftPhase = false;
  fsLfr = fsLfl = fsLrr = fsLrl = 0.1;
  fsRfr = fsRfl = fsRrr = fsRrl = 0.1;
}

void BodyModel::setIsLeftPhase(bool b)
{
  isLeftPhase = b;
}

float ABS(float x)
{
  return (x > 0 ? x : -x);
}

void BodyModel::updateZMPL(SensorValues &sensors)
{
  float temp = sensors.sensors[Sensors::LFoot_FSR_FrontLeft];
  if (fsLfl < temp and fsLfl < 5.0)
    fsLfl = temp;
  temp = sensors.sensors[Sensors::LFoot_FSR_FrontLeft];
  if (fsLfl < temp and fsLfl < 5.0)
    fsLfl = temp;
  temp = sensors.sensors[Sensors::LFoot_FSR_FrontRight];
  if (fsLfr < temp and fsLfr < 5.0)
    fsLfr = temp;
  temp = sensors.sensors[Sensors::LFoot_FSR_RearLeft];
  if (fsLrl < temp and fsLrl < 5.0)
    fsLrl = temp;
  temp = sensors.sensors[Sensors::LFoot_FSR_RearRight];
  if (fsLrr < temp and fsLrr < 5.0)
    fsLrr = temp;
  temp = sensors.sensors[Sensors::RFoot_FSR_FrontLeft];
  if (fsRfl < temp and fsRfl < 5.0)
    fsRfl = temp;
  temp = sensors.sensors[Sensors::RFoot_FSR_FrontRight];
  if (fsRfr < temp and fsRfr < 5.0)
    fsRfr = temp;
  temp = sensors.sensors[Sensors::RFoot_FSR_RearLeft];
  if (fsRrl < temp and fsRrl < 5.0)
    fsRrl = temp;
  temp = sensors.sensors[Sensors::RFoot_FSR_RearRight];
  if (fsRrr < temp and fsRrr < 5.0)
    fsRrr = temp;
  lastZMPL = ZMPL;
  ZMPL = 0;
  // Calculate ZMPL (left-right) used to eg switch support foot in Walk2014
  float pressureL = +sensors.sensors[Sensors::LFoot_FSR_FrontLeft] / fsLfl
      + sensors.sensors[Sensors::LFoot_FSR_FrontRight] / fsLfr
      + sensors.sensors[Sensors::LFoot_FSR_RearLeft] / fsLrl
      + sensors.sensors[Sensors::LFoot_FSR_RearRight] / fsLrr;
  float pressureR = +sensors.sensors[Sensors::RFoot_FSR_FrontLeft] / fsRfl
      + sensors.sensors[Sensors::RFoot_FSR_FrontRight] / fsRfr
      + sensors.sensors[Sensors::RFoot_FSR_RearLeft] / fsRrl
      + sensors.sensors[Sensors::RFoot_FSR_RearRight] / fsRrr;
  float totalPressure = pressureL + pressureR;
  if (ABS(totalPressure) > 0.000001f)
  {
    ZMPL = (.080 * sensors.sensors[Sensors::LFoot_FSR_FrontLeft] / fsLfl
        + .030 * sensors.sensors[Sensors::LFoot_FSR_FrontRight] / fsLfr
        + .080 * sensors.sensors[Sensors::LFoot_FSR_RearLeft] / fsLrl
        + .030 * sensors.sensors[Sensors::LFoot_FSR_RearRight] / fsLrr
        - .030 * sensors.sensors[Sensors::RFoot_FSR_FrontLeft] / fsRfl
        - .080 * sensors.sensors[Sensors::RFoot_FSR_FrontRight] / fsRfr
        - .030 * sensors.sensors[Sensors::RFoot_FSR_RearLeft] / fsRrl
        - .080 * sensors.sensors[Sensors::RFoot_FSR_RearRight] / fsRrr)
        / totalPressure;
  }
}

