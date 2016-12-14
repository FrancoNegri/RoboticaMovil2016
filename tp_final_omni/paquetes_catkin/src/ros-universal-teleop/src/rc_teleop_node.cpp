#include <ros/ros.h>
#include "rc_teleop.h"

int main(int argc, char** argv)
{
  ros::init(argc, argv, "rc_teleop");

  universal_teleop::RCTeleop t;

  ros::spin();
}
