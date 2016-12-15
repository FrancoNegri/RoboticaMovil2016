#include <lazo_abierto/FeedForwardController.h>

FeedForwardController::FeedForwardController(ros::NodeHandle& nh) :
  TrajectoryFollower(nh)
{}

double lineal_interp(const ros::Time& t0, const ros::Time& t1, double y0, double y1, const ros::Time& t)
{
  return y0 + (t - t0).toSec() * (y1 - y0) / (t1 - t0).toSec();
}

bool FeedForwardController::control(const ros::Time& t, double& v, double& w, double &tita)
{
  size_t next_point_idx;

  if( not nextPointIndex(t, next_point_idx ) )
    return false;

  ROS_INFO_STREAM("processing index: " << next_point_idx);

  const robmovil_msgs::TrajectoryPoint& prev_point = getTrajectory().points[ next_point_idx-1 ];
  const robmovil_msgs::TrajectoryPoint& next_point = getTrajectory().points[ next_point_idx ];

  const ros::Time& t0 = getInitialTime() + prev_point.time_from_start;
  const ros::Time& t1 = getInitialTime() + next_point.time_from_start;

  assert(t0 <= t);
  assert(t < t1);

  double vx0 = prev_point.velocity.linear.x;
  double vx1 = next_point.velocity.linear.x;

  double vy0 = prev_point.velocity.linear.y;
  double vy1 = next_point.velocity.linear.y;

  double va0 = prev_point.velocity.angular.z;
  double va1 = next_point.velocity.angular.z;

  ROS_INFO_STREAM("inter: " << t0 << " " << t1 << " " << vx0 << " " << vx1 << " " << va0 << " " << vx1 << " " << t);

  double vx = lineal_interp(t0, t1, vx0, vx1, t);
  double vy = lineal_interp(t0, t1, vy0, vy1, t);
  double va = lineal_interp(t0, t1, va0, va1, t);

  v = sqrt(vx*vx + vy*vy);
  w = va;

  return true;
}
