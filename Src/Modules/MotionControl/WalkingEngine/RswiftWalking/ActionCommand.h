/*
 * ActionCommand.hpp
 *
 *  Created on: 2016年4月8日
 *      Author: carrol
 */

#pragma once

#include <stdint.h>
#include "Tools/Enum.h"
#include <iostream>
#include "Tools/Joints.h"

/* Remember to update your constants in python/wrappers/ActionCommand */

namespace ActionCommand
{

  /**
   * Command for controlling the body
   * Note: Some ActionType Commands WILL disable the head
   */
  struct Body
  {
    // Predefined actions. These take precedence over walk parameters
    ENUM( ActionType,
        { ,
          NONE = 0,

          // Stand - common starting pose for all other actions
          STAND,//1

          // Walk
          WALK,//2
          DRIBBLE,//3                           wwwwww

          // Actions
          GETUP_FRONT, GETUP_BACK,//4,5                   wwwwww
          KICK,//6
          INITIAL,//7
          DEAD, REF_PICKUP,//8, 9
          OPEN_FEET,//10
          THROW_IN,//11
          GOALIE_SIT,//12
          GOALIE_DIVE_RIGHT,//13
          GOALIE_DIVE_LEFT,//14
          GOALIE_CENTRE,//15              wwwwww
          GOALIE_UNCENTRE,//16
          GOALIE_STAND,//17
          GOALIE_INITIAL,//18
          GOALIE_AFTERSIT_INITIAL,//19
          DEFENDER_CENTRE,//20           wwwwww
          GOALIE_FAST_SIT,//21             wwwww
          GOALIE_PICK_UP,//22
          MOTION_CALIBRATE,//23
          STAND_STRAIGHT,//24
          LINE_UP,//25                                 wwwww
          // NUM_ACTION_TYPES
        })
      ;
      ActionType actionType;

      // Walk/Kick Parameters
      int forward;// How far forward (negative for backwards)  (mm)
      int left;// How far to the left (negative for rightwards) (mm)
      float turn;// How much anti-clockwise turn (negative for clockwise) (rad)
      float power;// How much kick power (0.0-1.0)
      float bend;

      // Kick parameters
      float speed;
      float kickDirection;

      enum Foot
      {
        LEFT = 0,
        RIGHT
      };
      Foot foot;
      bool isFast;

      // Set this to true if you want the robot to do a kick that tries to kick it not straight
      // but angled, primarily to avoid an opponent straight ahead of it.
      bool misalignedKick;

      /**
       * Constructor for walks and kicks
       * @param at Action Type
       * @param f  How far forward (mm)
       * @param l  How far to the left (mm)
       * @param t  How much counter-clockwise turn (rad)
       * @param p  How much power
       * @param bend  Angle to bend knees (rad)
       * @param s  How much speed
       * @param k  Direction to kick (rad)
       * @param ft  Which foot to use
       * @param fast  go fast or not
       * @see http://runswift.cse.unsw.edu.au/confluence/display/rc2010/Movement%2C+walk%2C+kicks
       */
      Body(ActionType at, int f = 0, int l = 0, float t = 0.0, float p = 1.0,
          float bend = 15.0, float s = 1.0, float k = 0.0, Foot ft = LEFT, bool fast=false,
          bool misalignedKick=false)
      : actionType(at),
        forward(f),
        left(l),
        turn(t),
        power(p),
        bend(bend),
        speed(s),
        kickDirection(k),
        foot(ft),
        isFast(fast),
        misalignedKick(misalignedKick)
        {}

        /* Boost python makes using default arguements difficult.
         * Define an arguementless constructor to wrap
         */
        Body()
        : actionType(NONE),
        forward(0),
        left(0),
        turn(0),
        power(0),
        bend(0),
        speed(0),
        kickDirection(0),
        foot(LEFT),
        isFast(false),
        misalignedKick(false)
        {}

        /*template<class Archive>
         void serialize(Archive &ar, const unsigned int file_version) {
         ar & actionType;
         ar & forward;
         ar & left;
         ar & turn;
         ar & power;
         }
         */
      };

  const uint8_t priorities[Body::numOfActionTypes] =
  { 0,  // NONE
      0,  // STAND
      0,  // WALK
      0,  // DRIBBLE
      2,  // GETUP_FRONT
      2,  // GETUP_BACK
      0,  // KICK
      2,  // INITIAL
      1,  // DEAD
      0,  // REF_PICKUP
      0,  // OPEN_FEET
      0,  // THROW_IN
      2,  // GOALIE_SIT
      2,  // GOALIE_DIVE_LEFT
      2,  // GOALIE_DIVE_RIGHT
      2,  // GOALIE_CENTRE
      2,  // GOALIE_UNCENTRE
      0,  // GOALIE_STAND
      0,  // GOALIE_INITIAL
      0,  // GOALIE_AFTERSIT_INITIAL
      2,  // DEFENDER_CENTRE
      2,  // GOALIE FAST SIT
      2,  // GOALIE_PICK_UP
      0,  // MOTION_CALIBRATE
      0,  // STAND_STRAIGHT
      0  // LINE_UP
      };

}
;
// namespace ActionCommand
