/** A test striker option without common decision */
option(Striker)
{
  initial_state(start)
  {
    transition
    {
      if(state_time > 1000)
        goto turnToBall;
    }
    action
    {
      theHeadControlMode = HeadControl::lookForward;
      Stand();
    }
  }

  state(turnToBall)
  {
    transition
    {
      if(libCodeRelease.timeSinceBallWasSeen() > 7000)
        goto searchForBall;
      if(std::abs(theBallModel.estimate.position.angle()) < 5_deg)
        goto walkToBall;
    }
    action
    {
      theHeadControlMode = HeadControl::lookForward;
      WalkToTarget(Pose2f(50.f, 50.f, 50.f), Pose2f(theBallModel.estimate.position.angle(), 0.f, 0.f));
    }
  }

  state(walkToBall)
  {
    transition
    {
      if(libCodeRelease.timeSinceBallWasSeen() > 7000)
        goto searchForBall;
      if(theBallModel.estimate.position.norm() < 500.f)
        goto alignToGoal;
    }
    action
    {
      theHeadControlMode = HeadControl::lookForward;
      WalkToTarget(Pose2f(50.f, 50.f, 50.f), theBallModel.estimate.position);
    }
  }

  state(alignToGoal)
  {
    transition
    {
      if(libCodeRelease.timeSinceBallWasSeen() > 7000)
        goto searchForBall;
      if(std::abs(libCodeRelease.angleToGoal) < 10_deg && std::abs(theBallModel.estimate.position.y()) < 100.f)
        goto alignBehindBall;
    }
    action
    {
      theHeadControlMode = HeadControl::lookForward;
      WalkToTarget(Pose2f(100.f, 100.f, 100.f), Pose2f(libCodeRelease.angleToGoal, theBallModel.estimate.position.x() - 400.f, theBallModel.estimate.position.y()));
    }
  }

  state(alignBehindBall)
  {
    transition
    {
      if(libCodeRelease.timeSinceBallWasSeen() > 7000)
        goto searchForBall;
      if(libCodeRelease.between(theBallModel.estimate.position.y(), 30.f, 60.f)
          && libCodeRelease.between(theBallModel.estimate.position.x(), 120.f, 160.f)
          && std::abs(libCodeRelease.angleToGoal) < 5_deg)
        goto kick;
    }
    action
    {
      theHeadControlMode = HeadControl::lookForward;
      WalkToTarget(Pose2f(80.f, 80.f, 80.f), Pose2f(libCodeRelease.angleToGoal, theBallModel.estimate.position.x() - 140.f, theBallModel.estimate.position.y() - 45.f));
    }
  }

  state(kick)
  {
    transition
    {
      if(state_time > 10000)
        goto start;
    }
    action
    {
      Annotation("Alive and Kickin'");
      theHeadControlMode = HeadControl::lookForward;
      KickForward(KickRequest::kickForward,true);
    }
  }
  
  state(searchForBall)
  {
    transition
    {
      if(libCodeRelease.timeSinceBallWasSeen() < 300)
        goto turnToBall;
    }
    action
    {
      theHeadControlMode = HeadControl::lookForward;    
      WalkAtSpeedPercentage(Pose2f(1.f, 0.f, 0.f));
    }
  }
}
