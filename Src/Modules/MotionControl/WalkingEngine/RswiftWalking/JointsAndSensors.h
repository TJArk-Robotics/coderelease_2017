/*
 * JointsAndSensors.hpp
 *
 *  Created on: 2016年4月13日
 *      Author: carrol
 */

#pragma once

#include <iostream>
#include <cmath>
#include "Tools/Enum.h"
#include "Representations/Infrastructure/JointAngles.h"
#include "Representations/Infrastructure/SensorData/InertialSensorData.h"
#include "Representations/Infrastructure/SensorData/FsrSensorData.h"

using namespace std;

float DEG2RAD(float x);
float RAD2DEG(float x);

//namespace cJoints
//{
// ENUM(Joint,
//{,
//    headYaw,
//    headPitch,
//
//    lShoulderPitch,
//    lShoulderRoll,
//
//    lElbowYaw,
//    lElbowRoll,
//
//    lWristYaw,
//    lHand, //< not an Angle, instead %
//
//    lHipYawPitch,
//    lHipRoll,
//    lHipPitch,
//
//    lKneePitch,
//
//    lAnklePitch,
//    lAnkleRoll,
//
//	 //< not a joint in the real nao, same as lHipYawPitch i suppose--Carrol
//	//rHipYawPitch,                //= lHipYawPitch,
//    rHipRoll,
//    rHipPitch,
//
//    rKneePitch,
//
//    rAnklePitch,
//    rAnkleRoll,
//
//    rShoulderPitch,
//    rShoulderRoll,
//
//    rElbowYaw,
//    rElbowRoll,
//
//    rWristYaw,
//    rHand, //< not an Angle, instead %
//
//	//numOfJoints,    //default provided by macro Enum
//});

// const std::string pragmafliteJointNames[] = {
//    "Head Yaw",
//    "Head Pitch",
//    "L Shoulder Pitch",
//    "L Shoulder Roll",
//    "L Elbow Yaw",
//    "L Elbow Roll",
//    "L Wrist Yaw",
//    "L Hand",
//    "L Hip Yaw Pitch",
//    "L Hip Roll",
//    "L Hip Pitch",
//    "L Knee Pitch",
//    "L Ankle Pitch",
//    "L Ankle Roll",
//    "R Hip Roll",
//    "R Hip Pitch",
//    "R Knee Pitch",
//    "R Ankle Pitch",
//    "R Ankle Roll",
//    "R Shoulder Pitch",
//    "R Shoulder Roll",
//    "R Elbow Yaw",
//    "R Elbow Roll",
//    "R Wrist Yaw",
//    "R Hand",
// };
//
// const float HeadYaw_Min          = DEG2RAD(-119.5);
// const float HeadYaw_Max          = DEG2RAD(119.5);
// const float HeadPitch_Min        = DEG2RAD(-38.5);
// const float HeadPitch_Max        = DEG2RAD(29.5);
//
// const float LShoulderPitch_Min   = DEG2RAD(-119.5);
// const float LShoulderPitch_Max   = DEG2RAD(119.5);
// const float LShoulderRoll_Max    = DEG2RAD(76);
// const float LShoulderRoll_Min    = DEG2RAD(-18);
// const float LElbowYaw_Min        = DEG2RAD(-119.5);
// const float LElbowYaw_Max        = DEG2RAD(119.5);
// const float LElbowRoll_Min       = DEG2RAD(-88.5);
// const float LElbowRoll_Max       = DEG2RAD(-2);
// const float LHand_Min            = 0.0;
// const float LHand_Max            = 1.0;
// const float LWristYaw_Min        = DEG2RAD(-104.5);
// const float LWristYaw_Max        = DEG2RAD(104.5);
//
// const float LHipYawPitch_Min     = DEG2RAD(-65.62);
// const float LHipYawPitch_Max     = DEG2RAD(42.44);
// const float LHipPitch_Min        = DEG2RAD(-101.63);
// const float LHipPitch_Max        = DEG2RAD(27.73);
// const float LHipRoll_Min         = DEG2RAD(-21.74);
// const float LHipRoll_Max         = DEG2RAD(45.29);
// const float LKneePitch_Min       = DEG2RAD(-5.29);
// const float LKneePitch_Max       = DEG2RAD(121.04);
// const float LAnklePitch_Min      = DEG2RAD(-68.15);
// const float LAnklePitch_Max      = DEG2RAD(52.86);
// const float LAnkleRoll_Min       = DEG2RAD(-22.79);
// const float LAnkleRoll_Max       = DEG2RAD(44.06);
//
// const float RHipPitch_Min        = DEG2RAD(-101.54);
// const float RHipPitch_Max        = DEG2RAD(27.82);
// const float RHipRoll_Min         = DEG2RAD(-42.30);
// const float RHipRoll_Max         = DEG2RAD(23.76);
// const float RKneePitch_Min       = DEG2RAD(-5.90);
// const float RKneePitch_Max       = DEG2RAD(121.47);
// const float RAnklePitch_Min      = DEG2RAD(-67.97);
// const float RAnklePitch_Max      = DEG2RAD(53.40);
// const float RAnkleRoll_Min       = DEG2RAD(-45.03);
// const float RAnkleRoll_Max       = DEG2RAD(22.27);
//
// const float RShoulderPitch_Min   = DEG2RAD(-119.5);
// const float RShoulderPitch_Max   = DEG2RAD(119.5);
// const float RShoulderRoll_Min    = DEG2RAD(-76);
// const float RShoulderRoll_Max    = DEG2RAD(18);
// const float RElbowYaw_Min        = DEG2RAD(-119.5);
// const float RElbowYaw_Max        = DEG2RAD(119.5);
// const float RElbowRoll_Min       = DEG2RAD(2);
// const float RElbowRoll_Max       = DEG2RAD(88.5);
// const float RWristYaw_Min        = DEG2RAD(-104.5);
// const float RWristYaw_Max        = DEG2RAD(104.5);
// const float RHand_Min            = 0.0;
// const float RHand_Max            = 1.0;
//
// const float limitRadians[25][2] ={
// 		   {HeadYaw_Min, HeadYaw_Max},
// 		   {HeadPitch_Min,HeadPitch_Max},
//
// 		   {LShoulderPitch_Min,LShoulderPitch_Max},
// 		   {LShoulderRoll_Min, LShoulderRoll_Max},
// 		   {LElbowYaw_Min, LElbowYaw_Max},
// 		   {LElbowRoll_Min,LElbowRoll_Max},
// 		   {LWristYaw_Min,LWristYaw_Max},
// 		   {LHand_Min, LHand_Max},
//
// 		   {LHipYawPitch_Min, LHipYawPitch_Max},
// 		   {LHipRoll_Min, LHipRoll_Max},
// 		   {LHipPitch_Min, LHipPitch_Max},
// 		   {LKneePitch_Min,LKneePitch_Max},
// 		   {LAnklePitch_Min, LAnklePitch_Max},
// 		   {LAnkleRoll_Min, LAnkleRoll_Max},
//
// 		   {RHipRoll_Min, RHipRoll_Max},
// 		   {RHipPitch_Min, RHipPitch_Max},
// 		   {RKneePitch_Min, RKneePitch_Max},
// 		   {RAnklePitch_Min, RAnklePitch_Max},
// 		   {RAnkleRoll_Min, RAnkleRoll_Max},
//
// 		   {RShoulderPitch_Min, RShoulderPitch_Max},
// 		   {RShoulderRoll_Min, RShoulderRoll_Max},
// 		   {RElbowYaw_Min, RElbowYaw_Max},
// 		   {RElbowRoll_Min, RElbowRoll_Max},
// 		   {RWristYaw_Min, RWristYaw_Max},
// 		   {RHand_Min, RHand_Max}
//    };
//};


namespace Sensors
{
  /**
   * foot FSR positions relative to ankle frame
   * 1st index specifies left or right, 2nd index specifies which of 4,
   * 3rd index specifies x or y axis
   */
  const float FSR_POS[2][4][2] =
  {
  {
  { 70.25, 29.9 },
  { 70.25, -23.1 },
  { -30.25, 29.9 },
  { -29.65, -19.1 } },
  {
  { 70.25, 23.1 },
  { 70.25, -29.9 },
  { -30.25, 19.1 },
  { -29.65, -29.9 } } };
  ENUM(Sensor, {, InertialSensor_AccX = 0, InertialSensor_AccY,
       InertialSensor_AccZ, InertialSensor_GyrX, InertialSensor_GyrY,
       InertialSensor_GyrRef, InertialSensor_AngleX, InertialSensor_AngleY,

       InertialSensor_GyroscopeX,
       InertialSensor_GyroscopeY, InertialSensor_GyroscopeZ,
       InertialSensor_AccelerometerX, InertialSensor_AccelerometerY,
       InertialSensor_AccelerometerZ,

       LFoot_FSR_FrontLeft,
       LFoot_FSR_FrontRight, LFoot_FSR_RearLeft, LFoot_FSR_RearRight,
       LFoot_FSR_CenterOfPressure_X, LFoot_FSR_CenterOfPressure_Y,

       RFoot_FSR_FrontLeft,
       RFoot_FSR_FrontRight, RFoot_FSR_RearLeft, RFoot_FSR_RearRight,
       RFoot_FSR_CenterOfPressure_X, RFoot_FSR_CenterOfPressure_Y,

       LFoot_Bumper_Left,
       LFoot_Bumper_Right, RFoot_Bumper_Left, RFoot_Bumper_Right,

       ChestBoard_Button,

       Head_Touch_Front,
       Head_Touch_Rear, Head_Touch_Middle,

       Battery_Charge,
       Battery_Current, US,

       //NUMBER_OF_SENSORS,   // default as above
       });

  const std::string sensorNames[] =
  { "InertialSensor/AccX", "InertialSensor/AccY", "InertialSensor/AccZ",
      "InertialSensor/GyrX", "InertialSensor/GyrY", "InertialSensor/GyrRef",
      "InertialSensor/AngleX", "InertialSensor/AngleY",

      "InertialSensor/GyroscopeX", "InertialSensor/GyroscopeY",
      "InertialSensor/GyroscopeZ", "InertialSensor/AccelerometerX",
      "InertialSensor/AccelerometerY", "InertialSensor/AccelerometerZ",

      "LFoot/FSR/FrontLeft", "LFoot/FSR/FrontRight", "LFoot/FSR/RearLeft",
      "LFoot/FSR/RearRight", "LFoot/FSR/CenterOfPressure/X",
      "LFoot/FSR/CenterOfPressure/Y",

      "RFoot/FSR/FrontLeft", "RFoot/FSR/FrontRight", "RFoot/FSR/RearLeft",
      "RFoot/FSR/RearRight", "RFoot/FSR/CenterOfPressure/X",
      "RFoot/FSR/CenterOfPressure/Y",

      "LFoot/Bumper/Left", "LFoot/Bumper/Right", "RFoot/Bumper/Left",
      "RFoot/Bumper/Right",

      "ChestBoard/Button",

      "Head/Touch/Front", "Head/Touch/Rear", "Head/Touch/Middle",

      "Battery/Charge", "Battery/Current", "US" };
}


// Limits for Joints in RADIANS
namespace Radians {
   const float HeadYaw_Min          = DEG2RAD(-119.5);
   const float HeadYaw_Max          = DEG2RAD(119.5);
   const float HeadPitch_Min        = DEG2RAD(-38.5);
   const float HeadPitch_Max        = DEG2RAD(29.5);
   const float LShoulderPitch_Min   = DEG2RAD(-119.5);
   const float LShoulderPitch_Max   = DEG2RAD(119.5);
   const float LShoulderRoll_Max    = DEG2RAD(76);
   const float LShoulderRoll_Min    = DEG2RAD(-18);
   const float LElbowYaw_Min        = DEG2RAD(-119.5);
   const float LElbowYaw_Max        = DEG2RAD(119.5);
   const float LElbowRoll_Min       = DEG2RAD(-88.5);
   const float LElbowRoll_Max       = DEG2RAD(-2);
   const float LWristYaw_Min        = DEG2RAD(-104.5);
   const float LWristYaw_Max        = DEG2RAD(104.5);
   const float LHand_Min            = 0.0;
   const float LHand_Max            = 1.0;
   const float LHipYawPitch_Min     = DEG2RAD(-65.62);
   const float LHipYawPitch_Max     = DEG2RAD(42.44);
   const float LHipPitch_Min        = DEG2RAD(-101.63);
   const float LHipPitch_Max        = DEG2RAD(27.73);
   const float LHipRoll_Min         = DEG2RAD(-21.74);
   const float LHipRoll_Max         = DEG2RAD(45.29);
   const float LKneePitch_Min       = DEG2RAD(-5.29);
   const float LKneePitch_Max       = DEG2RAD(121.04);
   const float LAnklePitch_Min      = DEG2RAD(-68.15);
   const float LAnklePitch_Max      = DEG2RAD(52.86);
   const float LAnkleRoll_Min       = DEG2RAD(-22.79);
   const float LAnkleRoll_Max       = DEG2RAD(44.06);
   const float RHipPitch_Min        = DEG2RAD(-101.54);
   const float RHipPitch_Max        = DEG2RAD(27.82);
   const float RHipRoll_Min         = DEG2RAD(-42.30);
   const float RHipRoll_Max         = DEG2RAD(23.76);
   const float RKneePitch_Min       = DEG2RAD(-5.90);
   const float RKneePitch_Max       = DEG2RAD(121.47);
   const float RAnklePitch_Min      = DEG2RAD(-67.97);
   const float RAnklePitch_Max      = DEG2RAD(53.40);
   const float RAnkleRoll_Min       = DEG2RAD(-45.03);
   const float RAnkleRoll_Max       = DEG2RAD(22.27);
   const float RShoulderPitch_Min   = DEG2RAD(-119.5);
   const float RShoulderPitch_Max   = DEG2RAD(119.5);
   const float RShoulderRoll_Min    = DEG2RAD(-76);
   const float RShoulderRoll_Max    = DEG2RAD(18);
   const float RElbowYaw_Min        = DEG2RAD(-119.5);
   const float RElbowYaw_Max        = DEG2RAD(119.5);
   const float RElbowRoll_Min       = DEG2RAD(2);
   const float RElbowRoll_Max       = DEG2RAD(88.5);
   const float RWristYaw_Min        = DEG2RAD(-104.5);
   const float RWristYaw_Max        = DEG2RAD(104.5);
   const float RHand_Min            = 0.0;
   const float RHand_Max            = 1.0;
   const float RHipYawPitch_Min     = -3.14; // not a joint in real nao
   const float RHipYawPitch_Max     = 3.14;

   // Array to hold the maximum and minimum boundaries
   const float MaxAngle[] = {
      HeadYaw_Max,
      HeadPitch_Max,
      LShoulderPitch_Max,
      LShoulderRoll_Max,
      LElbowYaw_Max,
      LElbowRoll_Max,
      LWristYaw_Max,
      LHand_Max,
      RShoulderPitch_Max,
      RShoulderRoll_Max,
      RElbowYaw_Max,
      RElbowRoll_Max,
      RWristYaw_Max,
      RHand_Max,
      LHipYawPitch_Max,
      LHipRoll_Max,
      LHipPitch_Max,
      LKneePitch_Max,
      LAnklePitch_Max,
      LAnkleRoll_Max,
      RHipYawPitch_Max,
      RHipRoll_Max,
      RHipPitch_Max,
      RKneePitch_Max,
      RAnklePitch_Max,
      RAnkleRoll_Max
   };

   const float MinAngle[] = {
      HeadYaw_Min,
      HeadPitch_Min,
      LShoulderPitch_Min,
      LShoulderRoll_Min,
      LElbowYaw_Min,
      LElbowRoll_Min,
      LWristYaw_Min,
      LHand_Min,
      RShoulderPitch_Min,
      RShoulderRoll_Min,
      RElbowYaw_Min,
      RElbowRoll_Min,
      RWristYaw_Min,
      RHand_Min,
      LHipYawPitch_Min,
      LHipRoll_Min,
      LHipPitch_Min,
      LKneePitch_Min,
      LAnklePitch_Min,
      LAnkleRoll_Min,
      RHipYawPitch_Min,
      RHipRoll_Min,
      RHipPitch_Min,
      RKneePitch_Min,
      RAnklePitch_Min,
      RAnkleRoll_Min
   };
};
//
//
//// Motor speed reductions. These determine the maximum rotational
//// speed of the motors controlling the joint. It is 'highly' advised
//// that when controlling the motors these limits are taken into
//// consideration.  They have been placed in the same namespace as
//// their joint angle counter-parts as it makes sense given the
//// classification.
//// This is calculated as follows: No load speed / reduction ratio
//// / 60, then * 360 to get it into degrees as opposed to rotations.
//// Finally divide by 100 to get it into per 10ms. For example,
//// taking motor 1 type A,
//// (https://community.aldebaran-robotics.com/doc/1-14/family/nao_h25/motors_h25.html)
//// we get 8300 / 201.3 / 60 * 360 / 100. Since it's not a good idea
//// to push the motors to their limits, (overheating issues etc), we
//// only take 90% of this number, ie 2.23 (to 2 decimal places).
//
//// Motor limits in Radians
//namespace Radians {
//   // Maximum rotation speed (Radians) per cycle (10ms)
//   const float MOTOR1_REDUCTIONA_RAD = DEG2RAD(2.23);
//   const float MOTOR1_REDUCTIONB_RAD = DEG2RAD(3.43);
//   const float MOTOR2_REDUCTIONA_RAD = DEG2RAD(8.96);
//   const float MOTOR2_REDUCTIONB_RAD = DEG2RAD(12.52);
//   const float MOTOR3_REDUCTIONA_RAD = DEG2RAD(3.85);
//   const float MOTOR3_REDUCTIONB_RAD = DEG2RAD(3.34);
//
//   const float HeadPitchSpeed       = MOTOR3_REDUCTIONB_RAD;
//   const float HeadYawSpeed         = MOTOR3_REDUCTIONA_RAD;
//   const float ShoulderPitchSpeed   = MOTOR3_REDUCTIONA_RAD;
//   const float ShoulderRollSpeed    = MOTOR3_REDUCTIONB_RAD;
//   const float ElbowYawSpeed        = MOTOR3_REDUCTIONA_RAD;
//   const float ElbowRollSpeed       = MOTOR3_REDUCTIONB_RAD;
//   const float WristYawSpeed        = MOTOR2_REDUCTIONA_RAD;
//   const float HandSpeed            = MOTOR2_REDUCTIONB_RAD;
//   const float HipYawPitchSpeed     = MOTOR1_REDUCTIONA_RAD;
//   const float HipRollSpeed         = MOTOR1_REDUCTIONA_RAD;
//   const float HipPitchSpeed        = MOTOR1_REDUCTIONB_RAD;
//   const float KneePitchSpeed       = MOTOR1_REDUCTIONB_RAD;
//   const float AnklePitchSpeed      = MOTOR1_REDUCTIONB_RAD;
//   const float AnkleRollSpeed       = MOTOR1_REDUCTIONA_RAD;
//
//   const float MaxSpeed[] = {
//      HeadYawSpeed,
//      HeadPitchSpeed,
//      ShoulderPitchSpeed,  // Left arm
//      ShoulderRollSpeed,
//      ElbowYawSpeed,
//      ElbowRollSpeed,
//      WristYawSpeed,
//      HandSpeed,
//      HipYawPitchSpeed,    // Left leg
//      HipRollSpeed,
//      HipPitchSpeed,
//      KneePitchSpeed,
//      AnklePitchSpeed,
//      AnkleRollSpeed,
//      HipYawPitchSpeed,    // Right leg
//      HipRollSpeed,
//      HipPitchSpeed,
//      KneePitchSpeed,
//      AnklePitchSpeed,
//      AnkleRollSpeed,
//      ShoulderPitchSpeed,  // Right arm
//      ShoulderRollSpeed,
//      ElbowYawSpeed,
//      ElbowRollSpeed,
//      WristYawSpeed,
//      HandSpeed
//   };
//
//
//   /**
//    * Given the specified joint, caps it at its limits if the angle exceeds the boundary
//    * @param joint Joint to check the angle against
//    * @param angle Angle to check (in RADIANS)
//
//
//   static inline float limitJointRadians(   Joints::JointCode  joint, const float &angle) {
//      if (std::isnan(angle)) return 0.0;
//      if (angle > Radians::MaxAngle[joint]) return Radians::MaxAngle[joint];
//      if (angle < Radians::MinAngle[joint]) return Radians::MinAngle[joint];
//      return angle;
//   }
//    */
//
//};
//
//
///**
//* Limb Masses and Lengths
//* Taken from the Naoqi documentation. Limb names as Naoqi uses.
//*/
//
//namespace Limbs {
//const float NeckOffsetZ = 126.50;
//const float ShoulderOffsetY = 98.00;
//const float ElbowOffsetY = 15.00;
//const float UpperArmLength = 105.00;
//const float LowerArmLength = 55.95;
//const float ShoulderOffsetZ = 100.00;
//const float HandOffsetX = 57.75;
//const float HandOffsetZ = 12.31;
//const float HipOffsetZ = 85.00;
//const float HipOffsetY = 50.00;
//const float ThighLength = 100.00;
//const float TibiaLength = 102.90;
//const float FootHeight = 45.19;
//
//const float Length[] = {
//   NeckOffsetZ,
//   ShoulderOffsetY,
//   ElbowOffsetY,
//   UpperArmLength,
//   LowerArmLength,
//   ShoulderOffsetZ,
//   HandOffsetX,
//   HandOffsetZ,
//   HipOffsetZ,
//   HipOffsetY,
//   ThighLength,
//   TibiaLength,
//   FootHeight
//};
//
//const float TorsoMass = 1.04956;
//const float NeckMass = 0.06442; // Head Yaw
//const float HeadMass = 0.60533; // Head Pitch
//const float RightShoulderMass = 0.07504; // R Shoulder Pitch
//const float RightBicepMass = 0.15794; // R Shoulder Roll
//const float RightElbowMass = 0.06483; // R Elbow Yaw
//const float RightForearmMass = 0.07778; // R Elbow Roll
//const float RightHandMass = 0.18533; // R Wrist Yaw
//const float RightPelvisMass = 0.07118; // R Hip Yaw Pitch
//const float RightHipMass = 0.13053; // R Hip Roll
//const float RightThighMass = 0.38976; // R Hip Pitch
//const float RightTibiaMass = 0.29163; // R Knee Pitch
//const float RightAnkleMass = 0.13415; // R Ankle Pitch
//const float RightFootMass = 0.16171; // R Ankle Roll
//const float LeftShoulderMass = 0.07504; // L Shoulder Pitch
//const float LeftBicepMass = 0.15777; // L Shoulder Roll
//const float LeftElbowMass = 0.06483; // L Elbow Yaw
//const float LeftForearmMass = 0.07761; // L Elbow Roll
//const float LeftHandMass = 0.18533; // L Wrist Yaw
//const float LeftPelvisMass = 0.06981; // L Hip Yaw Pitch
//const float LeftHipMass = 0.13053; // L Hip Roll
//const float LeftThighMass = 0.38968; // L Hip Pitch
//const float LeftTibiaMass = 0.29142; // L Knee Pitch
//const float LeftAnkleMass = 0.13416; // L Ankle Pitch
//const float LeftFootMass = 0.16184; // L Ankle Roll
//
//const float Mass[] = {
//   TorsoMass,
//   NeckMass,
//   HeadMass,
//   RightShoulderMass,
//   RightBicepMass,
//   RightElbowMass,
//   RightForearmMass,
//   RightHandMass,
//   RightPelvisMass,
//   RightHipMass,
//   RightThighMass,
//   RightTibiaMass,
//   RightAnkleMass,
//   RightFootMass,
//   LeftShoulderMass,
//   LeftBicepMass,
//   LeftElbowMass,
//   LeftForearmMass,
//   LeftHandMass,
//   LeftPelvisMass,
//   LeftHipMass,
//   LeftThighMass,
//   LeftTibiaMass,
//   LeftAnkleMass,
//   LeftFootMass,
//};
//
//// Location of centre of masses in mm. Note: these are converted from m
//// so that they match the unit of measurement of the lengths of the links
//// (see above).
//const float TorsoCoM[] = {-4.13, 0.09, 43.42, 1};
//const float NeckCoM[] = {-0.01, 0.14, -27.42, 1};
//const float HeadCoM[] = {-1.12, 0.03, 52.58, 1};
//const float RightShoulderCoM[] = {-1.65, 26.63, 0.14, 1};
//const float RightBicepCoM[] = {24.29, -9.52, 3.20, 1};
//const float RightElbowCoM[] = {-27.44, 0.00, -0.14, 1};
//const float RightForearmCoM[] = {25.52, -2.81, 0.90, 1};
//const float RightHandCoM[] = {34.34, -0.88, 3.08, 1};
//const float RightPelvisCoM[] = {-7.66, 12.00, 27.16, 1};
//const float RightHipCoM[] = {-15.49, -0.29, -5.16, 1};
//const float RightThighCoM[] = {1.39, -2.25, -53.74, 1};
//const float RightTibiaCoM[] = {3.94, -2.21, -49.38, 1};
//const float RightAnkleCoM[] = {0.45, -0.30, 6.84, 1};
//const float RightFootCoM[] = {25.40, -3.32, -32.39, 1};
//const float LeftShoulderCoM[] = {-1.65, -26.63, 0.14, 1};
//const float LeftBicepCoM[] = {24.55, 5.63, 3.30, 1};
//const float LeftElbowCoM[] = {-27.44, 0.00, -0.14, 1};
//const float LeftForearmCoM[] = {25.56, 2.81, 0.76, 1};
//const float LeftHandCoM[] = {34.34, -0.88, 3.08, 1};
//const float LeftPelvisCoM[] = {-7.81, -11.14, 26.61, 1};
//const float LeftHipCoM[] = {-15.49, 0.29, -5.15, 1};
//const float LeftThighCoM[] = {1.38, 2.21, -53.73, 1};
//const float LeftTibiaCoM[] = {4.53, 2.25, -49.36, 1};
//const float LeftAnkleCoM[] = {0.45, 0.29, 6.85, 1};
//const float LeftFootCoM[] = {25.42, 3.30, -32.39, 1};
//};


/**
 * A container for joint angles & joint stiffnesses
 */
struct JointValues {
   /* Angles in radians. Used both for sensor reading and actuating */
   float angles[Joints::numOfJoints];
   /* Stiffnesses [-1.0, 0.0..1.0]. Used only for actuating */
   float stiffnesses[Joints::numOfJoints];
   /* Temperatures (estimated) in degrees Celcius. Used only for reading */
   float temperatures[Joints::numOfJoints];
   float currents[Joints::numOfJoints];
   JointValues()
   {
	   for (int i = 0; i < Joints::numOfJoints; ++i)
	   {
		   angles[i] = 0;
		   stiffnesses[i] = 0;
		   temperatures[i] = 0;
		   currents[i] = 0;
	   }
   }

   void getJointValues(const JointAngles &joint)
   {
	   for (int i = 0; i < Joints::numOfJoints; ++i)
	   {
		   //angles[i] = theJointAngles.angles[i] ;
		   angles[i] = joint.angles[i];
	   }
   }
};

using namespace Sensors;
//using namespace cJoints;

/**
* A container for joint values, IMU values, FSR values, buttons and sonar
* readings obtained from the Motion subsystem.
*/
struct SensorValues
{
  JointValues joints;
  float sensors[Sensors::numOfSensors];

  SensorValues()
  {
    for (int i = 0; i < Sensors::numOfSensors; ++i)
    {
      sensors[i] = 0.0f;
    }
  }
  void getSensorValues(const FsrSensorData &fsr, const InertialSensorData &gyro,
                       const JointAngles &joint)
  {
    joints.getJointValues(joint);
    sensors[Sensors::InertialSensor_GyroscopeZ] = gyro.gyro.z();
    sensors[Sensors::InertialSensor_GyrY] = gyro.gyro.y();
    sensors[Sensors::InertialSensor_GyrX] = gyro.gyro.x();

    sensors[Sensors::LFoot_FSR_FrontLeft] = fsr.left[0];
    sensors[Sensors::LFoot_FSR_FrontRight] = fsr.left[1];
    sensors[Sensors::LFoot_FSR_RearLeft] = fsr.left[2];
    sensors[Sensors::LFoot_FSR_RearRight] = fsr.left[3];

    sensors[Sensors::RFoot_FSR_FrontLeft] = fsr.right[0];
    sensors[Sensors::RFoot_FSR_FrontRight] = fsr.right[1];
    sensors[Sensors::RFoot_FSR_RearLeft] = fsr.right[2];
    sensors[Sensors::RFoot_FSR_RearRight] = fsr.right[3];
  }
};

