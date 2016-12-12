#include "pioneer_odometry.h"
#include <tf/tf.h>
#include <std_msgs/Float64.h>
#include <nav_msgs/Odometry.h>

using namespace robmovil;

#define WHEEL_RADIUS 50
#define ENCODER_TICKS 500.0
#define L_X 175
#define L_Y 175



PioneerOdometry::PioneerOdometry(ros::NodeHandle& nh)
  : nh_(nh), x_(0), y_(0), theta_(0), ticks_initialized_(false)
{
  // Nos suscribimos a los comandos de velocidad en el tópico "/robot/cmd_vel" de tipo geometry_msgs::Twist
  twist_sub_ = nh.subscribe("/robot/cmd_vel", 1, &PioneerOdometry::on_velocity_cmd, this);

  front_left_pub = nh.advertise<std_msgs::Float64>("/robot/front_left_wheel/cmd vel", 1);
  front_right_pub = nh.advertise<std_msgs::Float64>("/robot/front_right_wheel/cmd vel", 1);
  rear_left_pub = nh.advertise<std_msgs::Float64>("/robot/rear_left_wheel/cmd vel", 1);
  rear_right_pub = nh.advertise<std_msgs::Float64>("/robot/rear_right_wheel/cmd vel", 1);


  encoder_sub_ = nh.subscribe("/robot/encoders", 1, &PioneerOdometry::on_encoder_ticks, this);

  pub_odometry_ = nh.advertise<nav_msgs::Odometry>("/robot/odometry", 1);

  tf_broadcaster = boost::make_shared<tf::TransformBroadcaster>();
}

void PioneerOdometry::on_velocity_cmd(const geometry_msgs::Twist& twist)
{
  /** Completar los mensajes de velocidad */
    
  double linealVelX = twist.lineal.x;
  double linealVelY = twist.lineal.y;
  double angularVel = twist.angular.z;

  double vLeftFront = 1/WHEEL_RADIUS *(linealVelX - linealVelY - (L_X + L_Y)* angularVel);
  double vRightFront = 1/WHEEL_RADIUS *(linealVelX + linealVelY + (L_X + L_Y)* angularVel);
  double vLeftRear = 1/WHEEL_RADIUS *(linealVelX + linealVelY - (L_X + L_Y)* angularVel);
  double vRightRear = 1/WHEEL_RADIUS *(linealVelX - linealVelY + (L_X + L_Y)* angularVel);

  // publish left velocity
  {
    std_msgs::Float64 msg;
    msg.data = vLeftRear;
    rear_right_pub.publish( msg );
  }

  {
    std_msgs::Float64 msg;
    msg.data = vRightRear;
    rear_left_pub.publish( msg );
  }
  // publish right velocity
  {
    std_msgs::Float64 msg;
    msg.data = vRightFront;
    front_right_pub.publish( msg );
  }
  {
    std_msgs::Float64 msg;
    msg.data = vLeftFront;
    front_left_pub.publish( msg );
  }
}

void PioneerOdometry::on_encoder_ticks(const robmovil_msgs::MultiEncoderTicks& encoder)
{
  // La primera vez que llega un mensaje de encoders
  // inicializo las variables de estado.
  if (not ticks_initialized_) {
    ticks_initialized_ = true;
    last_ticks_left_rear = encoder.ticks[0];
    last_ticks_right_rear = encoder.ticks[1];
    last_ticks_left_front = encoder.ticks[2];
    last_ticks_right_front = encoder.ticks[3];
    last_ticks_time = encoder.header.stamp;
    return;
  }

  int32_t delta_ticks_left_rear = encoder.ticks[0] - last_ticks_left_rear;
  int32_t delta_ticks_right_rear = encoder.ticks[1] - last_ticks_right_rear;
  int32_t delta_ticks_left_front = encoder.ticks[2] - last_ticks_left_front;
  int32_t delta_ticks_right_front = encoder.ticks[3] - last_ticks_right_front;


  // calcular el desplazamiento relativo

  /* Utilizar este delta de tiempo entre momentos */
  double delta_t = (encoder.header.stamp - last_ticks_time).toSec();

  double velocidad_angular_left_rear = (delta_ticks_left_rear/ENCODER_TICKS)/delta_t;
  double velocidad_angular_right_rear = (delta_ticks_right_rear/ENCODER_TICKS)/delta_t;
  double velocidad_angular_left_front = (delta_ticks_left_front/ENCODER_TICKS)/delta_t;
  double velocidad_angular_right_front = (delta_ticks_right_front/ENCODER_TICKS)/delta_t;


  double velocidad_lineal_x = (velocidad_angular_right_front + velocidad_angular_left_front + velocidad_angular_right_rear + velocidad_angular_left_rear)*(WHEEL_RADIUS/4); 
  double velocidad_lineal_y = ( -velocidad_angular_right_front + velocidad_angular_left_front + velocidad_angular_right_rear - velocidad_angular_left_rear)*(WHEEL_RADIUS/4); 
  double velocidad_angular = ( -velocidad_angular_right_front + velocidad_angular_left_front - velocidad_angular_right_rear + velocidad_angular_left_rear)*(WHEEL_RADIUS/(4*(L_Y+L_X))); 

  /** Utilizar variables globales x_, y_, theta_ definidas en el .h */
  //x_ += ;
  //y_ += ;
  //theta_ += ;

  // Construir el mensaje odometry utilizando el esqueleto siguiente:
  nav_msgs::Odometry msg;

  msg.header.stamp = encoder.header.stamp;
  msg.header.frame_id = "odom";
  msg.child_frame_id = "base_link";

  msg.pose.pose.position.x = velocidad_lineal_x * delta_t ;
  msg.pose.pose.position.y = velocidad_lineal_y * delta_t ;
  msg.pose.pose.position.z = 0;

  msg.pose.pose.orientation = tf::createQuaternionMsgFromYaw(velocidad_angular * delta_t);

  pub_odometry_.publish( msg );

  // Actualizo las variables de estado

  last_ticks_left_ = encoder.ticks_left.data;
  last_ticks_right_ = encoder.ticks_right.data;
  last_ticks_time = encoder.header.stamp;

  /* Mando tambien un transform usando TF */
  tf::Transform t;
  tf::poseMsgToTF(msg.pose.pose, t);
  tf_broadcaster->sendTransform(tf::StampedTransform(t, encoder.header.stamp, "odom", "base_link"));

}
