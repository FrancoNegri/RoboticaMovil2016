#include <vector>
#include <numeric>      // std::accumulate
#include "imu_calibrator.h"

#define DO_CALIBRATION 1

robmovil_ekf::IMUCalibrator::IMUCalibrator(ros::NodeHandle& _n) : n(_n)
{
  imu_sub = n.subscribe("/imu", 1, &IMUCalibrator::on_imu_measurement, this);
  imu_calib_pub = n.advertise<sensor_msgs::Imu>("/imu_calib", 1);
  
  ros::NodeHandle nhp("~");
  int calibrating_time;
  nhp.param<int>("calibrate", calibrating_time, 0);

  if(calibrating_time){
    while( (ros::Time::now()) == ros::Time(0) ); // se espera hasta recibir el primer '/clock'
    timer_ = n.createTimer(ros::Duration(calibrating_time), &IMUCalibrator::calculate_bias, this, true);
    is_calibrating_ = true; // comenzar calibracion
  }else
    is_calibrating_ = false;
  
  bias_ = tf::Vector3(0,0,0);
  q_ = tf::createIdentityQuaternion();
}

void robmovil_ekf::IMUCalibrator::calculate_bias(const ros::TimerEvent& event){
  is_calibrating_ = false;
  
  // Calculamos el bias (media) de los datos almacenados para la calibración
  tf::Vector3 sum = std::accumulate(calibration_data_.begin(), calibration_data_.end(), tf::Vector3(0,0,0));
  bias_ = sum / calibration_data_.size();
  
  ROS_INFO_STREAM("bias: " << " " << bias_.getX() << " " << bias_.getY() << " " << bias_.getZ());
}

void robmovil_ekf::IMUCalibrator::on_imu_measurement(const sensor_msgs::Imu& msg)
{
  // Covertimos la velocidad angular al tipo tf::Vector3
  tf::Vector3 angular_velocity;
  tf::vector3MsgToTF(msg.angular_velocity, angular_velocity);
  
  // delta de tiempo entre mediciones
  double delta_t = (msg.header.stamp - time_last_measure_).toSec();
  time_last_measure_ = msg.header.stamp;

  if ( is_calibrating_ ) {
    ROS_INFO_ONCE("Calibrando IMU..");
    // Se acumulan las mediciones de imu para calcular el bias (la media)
    calibration_data_.push_back( angular_velocity );
  } else {
    // construimos el mensaje nuevo
    sensor_msgs::Imu imu_calib_msg;
    imu_calib_msg = msg;
    
    tf::Vector3 vel_sin_bias = angular_velocity - bias_; // removemos el bias de la velocidad angular
    tf::vector3TFToMsg(vel_sin_bias, imu_calib_msg.angular_velocity);
    
    ROS_INFO_STREAM("vel: " << vel_sin_bias.getZ());
    
    q_ *= tf::createQuaternionFromYaw(vel_sin_bias.getZ() * delta_t); // se asume velocidad constante en ese intervalo de tiempo
    q_.normalize();
    
    tf::quaternionTFToMsg(q_, imu_calib_msg.orientation);
    imu_calib_pub.publish(imu_calib_msg);
  }
}
