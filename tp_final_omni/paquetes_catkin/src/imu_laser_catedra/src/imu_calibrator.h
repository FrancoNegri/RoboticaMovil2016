#ifndef __ROBMOVIL_EKF_IMU_CALIBRATOR_H__
#define __ROBMOVIL_EKF_IMU_CALIBRATOR_H__

#include <ros/ros.h>
#include <tf/tf.h>
#include <sensor_msgs/Imu.h>

namespace robmovil_ekf {
  class IMUCalibrator
  {
    public:
      IMUCalibrator(ros::NodeHandle& n);

      void on_imu_measurement(const sensor_msgs::Imu& msg);

    private:

      void calculate_bias(const ros::TimerEvent &event);

      bool is_calibrating_;
      tf::Vector3 bias_;
      std::vector<tf::Vector3> calibration_data_;
      tf::Quaternion q_;

      ros::NodeHandle& n;

      ros::Timer timer_;

      ros::Subscriber imu_sub;
      ros::Publisher imu_calib_pub;
      ros::Time time_last_measure_;
  };
}

#endif // __ROBMOVIL_EKF_IMU_CALIBRATOR_H__
