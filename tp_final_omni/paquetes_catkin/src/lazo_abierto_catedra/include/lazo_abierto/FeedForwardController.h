#pragma once

#include "TrajectoryFollower.h"

class FeedForwardController : public TrajectoryFollower
{
  public:

    FeedForwardController(ros::NodeHandle& nh);

  protected:

    bool control(const ros::Time& t, double& v, double& w, double &tita) override;
};
