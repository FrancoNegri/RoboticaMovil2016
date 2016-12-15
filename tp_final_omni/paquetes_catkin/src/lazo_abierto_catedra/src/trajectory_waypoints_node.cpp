#include <ros/ros.h>
#include <trajectory_msgs/JointTrajectory.h>
#include <nav_msgs/Path.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PoseArray.h>
#include <tf/tf.h>
#include <vector>

struct waypoint {
  double time_from_start;
  double target_lin_vel;
  double target_ang_vel;
};

std::vector<waypoint> planBox()
{
  std::vector<waypoint> waypoints;
  
  waypoints.push_back({0, 0.25, 0});
  waypoints.push_back({8, 0.25, 0});
  waypoints.push_back({8, 0.25, -M_PI/4});
  waypoints.push_back({10, 0.25, 0});
  waypoints.push_back({15, 0, 0});
  
  return waypoints;
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "trajectory_waypoints");
  ros::NodeHandle nh;

  ros::Publisher trajectory_publisher = nh.advertise<trajectory_msgs::JointTrajectory>("/robot/trajectory", 1, true);
  
  // Path descripto en poses para visualizacion en RViz

  trajectory_msgs::JointTrajectory trajectory_msg;
  nav_msgs::Path path_msg;
  geometry_msgs::PoseArray poses_msg;

  trajectory_msg.header.seq = 0;
  trajectory_msg.header.stamp = ros::Time::now();
  trajectory_msg.header.frame_id = "odom";

  trajectory_msg.joint_names.push_back("x");
  trajectory_msg.joint_names.push_back("y");
  trajectory_msg.joint_names.push_back("theta");
  
  std::vector<waypoint> waypoints = planBox();

  for (waypoint wpoint : waypoints)
  {
    double t = wpoint.time_from_start;
    double vl = wpoint.target_lin_vel;
    double va = wpoint.target_ang_vel;

    trajectory_msgs::JointTrajectoryPoint point_msg;
    
    point_msg.time_from_start = ros::Duration(t);

    point_msg.velocities.push_back( vl );
    point_msg.velocities.push_back( va );

    trajectory_msg.points.push_back( point_msg );
  }

  trajectory_publisher.publish( trajectory_msg );

  ros::spin();

  return 0;
}
