#include <ros/ros.h>
#include "imu_calibrator.h"

int main(int argc, char** argv)
{
  ros::init(argc, argv, "imu_calibrator");
  ros::NodeHandle n;
  robmovil_ekf::IMUCalibrator imu_calibrator(n);
  ros::spin();
  return 0;
}
