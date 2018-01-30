/*
 * File:   KickForward.h
 * Author: wzf
 *
 * Created on 9/23/2015, 10:52
 */

//option(KickForward,(KickRequest::KickMotionID) kickMotionType, (bool)(false) mirror )
option (KickForward, ((KickRequest) KickMotionID) kickMotionType, (bool) (false) mirror)
{
  /** Set the motion request. */
  initial_state (setRequest)
  {
    transition
    {
      if (theMotionInfo.motion == MotionRequest::kick)
	goto requestIsExecuted;
    }
    action
    {
      theMotionRequest.motion = MotionRequest::kick;
      theMotionRequest.kickRequest.kickMotionType = kickMotionType;
      theMotionRequest.kickRequest.mirror = mirror;
    }
  }

  /** The motion process has started executing the request. */
  target_state (requestIsExecuted)
  {
    transition
    {
      if (theMotionInfo.motion != MotionRequest::kick)
	goto setRequest;
    }
    action
    {
      theMotionRequest.motion = MotionRequest::kick;
      theMotionRequest.kickRequest.kickMotionType = kickMotionType;
      theMotionRequest.kickRequest.mirror = mirror;
    }
  }
}
