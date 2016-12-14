#include <ros/ros.h>
#include "pioneer_odometry.h"

int main(int argc, char** argv)
{
  ros::init(argc, argv, "pioneer_odometry");
  ros::NodeHandle nh;
  robmovil::PioneerOdometry pioneer_odometry(nh);
  ros::spin();
  return 0;
}
