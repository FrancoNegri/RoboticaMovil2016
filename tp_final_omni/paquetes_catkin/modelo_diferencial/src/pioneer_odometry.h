#pragma once

#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <robmovil_msgs/EncoderTicks.h>
#include <tf/tf.h>
#include <tf/transform_broadcaster.h>

namespace robmovil
{

class PioneerOdometry
{
  public:

    PioneerOdometry(ros::NodeHandle& nh);

    void on_velocity_cmd(const geometry_msgs::Twist& twist);

    void on_encoder_ticks(const robmovil_msgs::EncoderTicks& encoder);

  private:

    ros::NodeHandle& nh_;

    ros::Subscriber twist_sub_, encoder_sub_;

    ros::Publisher front_left_pub, front_right_pub, rear_left_pub, rear_right_pub, pub_odometry_;

  // Acá pueden agregar las variables de instancia que necesiten
  // ...

    double x_, y_, theta_;

    bool ticks_initialized_;
    int32_t last_ticks_left_rear, last_ticks_right_rear, last_ticks_left_front, last_ticks_right_front;
    ros::Time last_ticks_time;

    boost::shared_ptr<tf::TransformBroadcaster> tf_broadcaster;
};

}
