#include <ros/ros.h>
#include <robmovil_msgs/Trajectory.h>
#include <robmovil_msgs/TrajectoryPoint.h>
#include <nav_msgs/Path.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PoseArray.h>
#include <tf/tf.h>
#include <angles/angles.h>

void build_sin_trajectory(double, double, double, double, robmovil_msgs::Trajectory&, nav_msgs::Path&);

int main(int argc, char** argv)
{
  ros::init(argc, argv, "trajectory_generator");
  ros::NodeHandle nh;
  ros::NodeHandle nhp("~");

  ros::Publisher trajectory_publisher = nh.advertise<robmovil_msgs::Trajectory>("/robot/trajectory", 1, true);
  
  // Path descripto en poses para visualizacion en RViz
  ros::Publisher path_publisher = nh.advertise<nav_msgs::Path>("/ground_truth/target_path", 1, true);

  robmovil_msgs::Trajectory trajectory_msg;
  nav_msgs::Path path_msg;

  trajectory_msg.header.seq = 0;
  trajectory_msg.header.stamp = ros::Time::now();
  trajectory_msg.header.frame_id = "map";

  path_msg.header.stamp = ros::Time::now();
  path_msg.header.frame_id = "map";
  
  double stepping;
  double total_time;
  double amplitude;
  double cycles;
  
  nhp.param<double>("stepping", stepping, 0.1);
  nhp.param<double>("total_time", total_time, 20); // 20: da masomenos, 50: lo sigue muy bien el pioneer
  nhp.param<double>("amplitude", amplitude, 1);
  nhp.param<double>("cycles", cycles, 1);
  
  build_sin_trajectory(stepping, total_time, amplitude, cycles, trajectory_msg, path_msg);

  trajectory_publisher.publish( trajectory_msg );
  path_publisher.publish( path_msg );

  ros::spin();

  return 0;
}

void build_sin_trajectory(double stepping, double total_time, double amplitude, double cycles, robmovil_msgs::Trajectory& trajectory_msg, nav_msgs::Path& path_msg)
{
  double initial_orientation = M_PI/2;
  
  for(int i = 0; i < 4; i++)
  {
    for (double t = 0; t <= total_time; t = t + stepping)
    {
      
      double x,y,vx,vy,a,va;
      if(i == 0){
	   x = -2+(4 * t/total_time);
	   y = 2;
	   vx = 4/total_time;
	   vy = 0;
      }
      if(i == 1){
	   x = 2;
	   y = 2-(4 * t/total_time);
	   vx = 0;
	   vy = -4/total_time;
      }
	if(i == 2){
	   x = 2-(4 * t/total_time);
	   y = -2;
	   vx = -4/total_time;
	   vy = 0;
	}
	if(i == 3){
	   x = -2;
	   y = -2 +(4 * t/total_time);
	   vx = 0;
	   vy = 4/total_time;
	}
      a = angles::normalize_angle(initial_orientation - (i * M_PI/2)  - ((M_PI/2)*t)/total_time);
      va = -(M_PI/2)/total_time;
      
      // derivadas segundas
      double vvx = 0;
      double vvy = 0;
      
      /* dado que la funcion esta construida pensada con Y "hacia arriba", X "hacia derecha" 
      * y la orientacion inicial puede no ser 0 -> entonces aplicamos una rotacion de manera de 
      * alinear la primera orientacion al eje X del robot y todo vector direccion de manera acorde */
//       double x_rot = cos(-initial_orientation) * x + -sin(-initial_orientation) * y;
//       double y_rot = sin(-initial_orientation) * x + cos(-initial_orientation) * y;
//       double vx_rot = cos(-initial_orientation) * vx + -sin(-initial_orientation) * vy;
//       double vy_rot = sin(-initial_orientation) * vx + cos(-initial_orientation) * vy;
//       double vvx_rot = cos(-initial_orientation) * vvx + -sin(-initial_orientation) * vvy;
//       double vvy_rot = sin(-initial_orientation) * vvx + cos(-initial_orientation) * vvy;
//       
//       x = x_rot; y = y_rot; vx = vx_rot; vy = vy_rot; vvx = vvx_rot; vvy = vvy_rot;
      
      // calculo del angulo en cada momento y la derivada del angulo

      // se crean los waypoints de la trajectoria
      robmovil_msgs::TrajectoryPoint point_msg;

      point_msg.time_from_start = ros::Duration( i*total_time + t );

      point_msg.transform.translation.x = x;
      point_msg.transform.translation.y = y;
      point_msg.transform.translation.z = 0;

      point_msg.transform.rotation = tf::createQuaternionMsgFromYaw( a );

      point_msg.velocity.linear.x = vx;
      point_msg.velocity.linear.y = vy;
      point_msg.velocity.linear.z = 0;

      point_msg.velocity.angular.x = 0;
      point_msg.velocity.angular.y = 0;
      point_msg.velocity.angular.z = va;

      trajectory_msg.points.push_back( point_msg );
      
      geometry_msgs::PoseStamped stamped_pose_msg;
      
      stamped_pose_msg.header.stamp = path_msg.header.stamp;
      stamped_pose_msg.header.frame_id = path_msg.header.frame_id;
      
      stamped_pose_msg.pose.position.x = x;
      stamped_pose_msg.pose.position.y = y;
      stamped_pose_msg.pose.position.z = 0;
      
      stamped_pose_msg.pose.orientation = tf::createQuaternionMsgFromYaw(a);
      
      path_msg.poses.push_back(stamped_pose_msg);
    }
  }   
  
  
}
