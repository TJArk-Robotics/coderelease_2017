/**
 * @file WalkingEngine.h
 * Declaration a module that creates the walking motions
 * @author Carrol Zhang, yongqi
 */

#pragma once

#include "WalkingEngineKicks.h"
#include "WalkingEngineTools.h"
#include "Representations/Configuration/DamageConfiguration.h"
#include "Representations/Configuration/MassCalibration.h"
#include "Representations/Configuration/RobotDimensions.h"
#include "Representations/Infrastructure/FrameInfo.h"
#include "Representations/MotionControl/HeadJointRequest.h"
#include "Representations/MotionControl/MotionInfo.h"
#include "Representations/MotionControl/MotionRequest.h"
#include "Representations/MotionControl/MotionSelection.h"
#include "Representations/MotionControl/ArmMotionInfo.h"
#include "Representations/MotionControl/ArmMotionRequest.h"
#include "Representations/MotionControl/ArmMotionSelection.h"
#include "Representations/MotionControl/WalkingEngineOutput.h"
#include "Representations/MotionControl/StandBodyRotation.h"
#include "Representations/Sensing/FallDownState.h"
#include "Representations/Sensing/GroundContactState.h"
#include "Representations/Sensing/InertialData.h"
#include "Representations/Sensing/RobotModel.h"
#include "Representations/Sensing/TorsoMatrix.h"
#include "Representations/Infrastructure/JointAngles.h"
#include "Representations/Infrastructure/SensorData/InertialSensorData.h"
#include "Representations/Infrastructure/SensorData/FsrSensorData.h"
#include "Representations/Modeling/BallModel.h"
#include "Tools/Range.h"
#include "Tools/RingBuffer.h"
#include "Tools/RingBufferWithSum.h"
#include "Tools/Module/Module.h"
#include "Tools/Optimization/ParticleSwarm.h"

#include "RswiftWalking/BodyModel.h"
#include "RswiftWalking/MotionOdometry.h"
#include "RswiftWalking/ActionCommand.h"
#include "RswiftWalking/XYZ_Coord.h"

#define TURN_THRESHOLD DEG2RAD(25) // min angle for triggering turn step
#define TURN_PERIOD 0.75f // time to perform turn step
#define TURN_STEP_HEIGHT 0.024f // how far to lift turning foot
#define TURN_LEAN DEG2RAD(20.25) // sideways lean while turning
#define TURN_SCALE 1.15f // how much to scale up turn
#define EPSILON 0.01f       //10 mm
#define TURN_EPSILON 0.05f  //2.8 degrees

//mm value of safety barrier
#define FOOT_AVOID_RADIUS 75
#define TURN_CLIPPING 0.10 //radians
#define NINETY 1.51f
#define FORTY_FIVE 1.51 / 2
#define TT_DEGREE FORTY_FIVE / 2

const float MM_PER_M  = 1000.0;                            // number of millimeters in one meter
const float CROUCH_STAND_PERIOD = 0.5;                    // time in seconds to crouch
const float COM_OFFSET = 0.01f;                             // center of mass offset in x direction in meters
const float FORWARD_CHANGE = 0.15f;                          // was 0.08. max change of 100mm/sec at each leg change to ratchet up/down
const float LEFT_CHANGE = 0.2f;
const float STAND_HIP_HEIGHT = 0.23f;//0.248;                      // for tall power saving stand, matches INITIAL action command
const float KNEE_PITCH_RANGE = DEG2RAD(60);                // the knee pitch range from standing to crouching
const float BASE_WALK_PERIOD = .25f; //.25;                 // seconds to walk one step, ie 1/2 walk cycle
const float WALK_HIP_HEIGHT = .23f;                         // Walk hip height - seems to work from .2 to .235
const float MAX_FORWARD = .3f;                              // meters
const float MAX_LEFT = .2f;                                 // meters
const float MAX_TURN = .8f;                                // radians
const float BASE_LEG_LIFT = 0.010f;                         // meters


MODULE(WalkingEngine,
{,
  USES(ArmMotionInfo),
  USES(JointRequest),
  USES(MotionInfo),
  REQUIRES(ArmMotionSelection),
  REQUIRES(DamageConfigurationBody),
  REQUIRES(FallDownState),
  REQUIRES(FrameInfo),
  REQUIRES(GroundContactState),
  REQUIRES(HeadJointRequest),
  REQUIRES(InertialData),
  REQUIRES(MassCalibration),
  REQUIRES(MotionRequest),
  REQUIRES(MotionSelection),
  REQUIRES(RobotDimensions),
  REQUIRES(RobotModel),
  REQUIRES(StandBodyRotation),
  REQUIRES(TorsoMatrix),
  REQUIRES(JointAngles),
  REQUIRES(InertialSensorData),
  REQUIRES(FsrSensorData),
  REQUIRES(BallModel),
  PROVIDES(WalkingEngineOutput),
  REQUIRES(WalkingEngineOutput),
  PROVIDES(StandOutput),
  LOADS_PARAMETERS(
  {,
    (float) offsetVal,
  }),
});

/**
* A module that creates walking motions using a three-dimensional linear inverted pendulum
*/
class WalkingEngine : public WalkingEngineBase
{
public:
  /**
  * Called from a MessageQueue to distribute messages
  * @param message The message that can be read
  * @return true if the message was handled
  */
  static bool handleMessage(InMessage& message);

  /**
  * Default constructor
  */
  WalkingEngine();

  /**
  * Destructor
  */
  ~WalkingEngine() {theInstance = 0;}

private:
  static PROCESS_LOCAL WalkingEngine* theInstance; /**< Points to the only instance of this class in this process or is 0 if there is none */

  ENUM(WalkOption,
  {,
    STAND        = 0, // with knees straight and stiffness set to zero to conserve energy and heat generation in motors
    STANDUP      = 1, // process of moving from WALK crouch to STAND
    CROUCH       = 2, // process of transitioning from STAND to WALK
    WALK         = 3,
    READY        = 4, // stand still ready to walk
    KICK         = 5,
    STEP         = 6,
    NONE         = 7,
    NUMBER_OF_WALK_OPTIONS,
  });

  ENUM(WalkState,
  {,
    WALKING        = 0,
    STARTING       = 1,
    STOPPING       = 2,
    NOT_WALKING    = 3,
    NUMBER_OF_WALK_STATES,
  });

  // Use for iterative inverse kinematics for turning (see documentation BH 2010)
  struct Hpr {
    float Hp;
    float Hr;
    // Hpr(): Hp(0.0f), Hr(0.0f) { }
  };

  /** Initialize derived parameters. */
  void init();

  void reset();

  /**
  * The central update method to generate the walking motion
  * @param walkingEngineOutput The WalkingEngineOutput (mainly the resulting joint angles)
  */
  void update(WalkingEngineOutput& walkingEngineOutput) override;

  /**
  * The update method to generate the standing stance
  * @param standOutput The WalkingEngineStandOutput (mainly the resulting joint angles)
  */
  void update(StandOutput& standOutput) override {static_cast<JointRequest&>(standOutput) = jointRequest;}


  void updateMotionRequest();

  JointValues makeJoints(float ballX, float ballY);

  void generateJointRequest(JointValues& j);

  Odometry updateOdometry(bool isLeftSwingFoot);

  void generateOutput(WalkingEngineOutput& WalkingEngineOutput);

  void generateDummyOutput(WalkingEngineOutput& WalkingEngineOutput);

  float leftAngle();

  float linearStep(float time, float period);

  float parabolicReturn(float f);

  float parabolicStep(float time, float period, float deadTimeFraction);

  float interpolateSmooth(float start, float end, float tCurrent, float tEnd); // sinusoidal interpolation

  // Foot to Body coord transform used to calculate IK foot position and ankle tilts to keep foot in ground plane when turning
  XYZ_Coord mf2b(float Hyp, float Hp, float Hr, float Kp, float Ap,
      float Ar, float xf, float yf, float zf, bool left);

  Hpr hipAngles(float Hyp, float Hp, float Hr, float Kp, float Ap,
      float Ar, float xf, float yf, float zf, XYZ_Coord e, bool left);

  WalkOption walkOption;
  WalkOption lastOption;
  WalkState walkState;

  JointRequest jointRequest;
  Pose2f requestedWalkSpeed;
  Pose2f requestedWalkTarget;
  Pose2f lastCopiedWalkTarget;
  WalkingEngineKicks kicks;

  BodyModel bodyModel;
  Odometry odometry;
  Odometry lastOdometry;
  SensorValues sensors;
  MotionOdometry motionOdometry;
  ActionCommand::Body active;

  Odometry odometrySum;
  int delay = 0;
  
  Vector3f lastOdometryOrigin = Vector3f::Zero();
  Pose2f odometryOffset;
  Pose3f lastFootLeft;
  Pose3f lastFootRight;

  // data for calculate
  bool exactStepsRequested;
//  bool v4;
  bool stopping;
  bool stopped;
  // time step, timers,
  float dt;
  float t;
  float globalTime;
  float timer;
  float T;                                                // period of half a walk cycle
  const float z;                                          // zero
  // 找到代码里面与utils/body.hpp比较相像的文件
  // Nao H25 V4 dimensions - from utils/body.hpp and converted to meters
  float thighL;                                            // thigh length in meters
  float thighR;                                            // thigh length in meters
  float tibia;                                            // tibia length in meters
  float ankle;                                            // height of ankle above ground
  // Walk 2014 parameters in meters and seconds
  float hiph;                                             // variable vertical distance ground to hip in meters
  float hiph0;                                            // some initial hiph
  float foothL;                                           // meters left foot is lifted off the ground
  float foothR;                                           // meters right foot is lifted off the ground
  float nextFootSwitchT;                                  // next time-point at which to change support foot
  float forward;                                          // Omnidirectional walk forward/backward
  float lastForward;                                      // previous forward value accepted
  float forwardL0, forwardL;                              // variable left foot position wrt standing
  float forwardR0, forwardR;                              // variable right foot position wrt standing
  float leftR;                                            // sideways step in meters for right foot
  float leftL;                                            // sideways step in meters for left  foot
  float left, lastLeft;                                   // Omnidirectional walk left/right
  float turn;                                             // Omnidirectional walk CW / CCW
  float power;                                            // Omnidirectional walk - reserved for kicking
  float bend;
  float speed;
  // body类的foot也有用
  ActionCommand::Body::Foot foot;                         // is this right?
  bool isFast;
  float stiffness;                                        // global stiffness (poweer to motors)
  float turnRL;                                           // turn variable
  float turnRL0;                                          // turnRL at support foot switch
  float swingAngle;                                       // recovery angle for sideStepping
  bool supportFoothasChanged;                             // Indicates that support foot is deemed to have changed
  bool lastIsLeftPhase;
  float balanceAdjustment;
  float coronalBalanceAdjustment;
  float comOffset;                                        // move in meters of CoM in x-direction when walking to spread weight more evenly over foot
  // Gyro filters
  float filteredGyroX;
  float filteredGyroY;

  float rock;
  float shoulderPitchL;                                   // to swing left  arm while walking / kicking
  float shoulderPitchR;                                   // to swing right arm while walking / kicking
  float shoulderRollL;
  float shoulderRollR;
  float turnAngle;
  float lastTurn;

  //for odometry updates
  float prevTurn;
  float prevForwardL;
  float prevForwardR;
  float prevLeftL;
  float prevLeftR;
  // end of data

};
