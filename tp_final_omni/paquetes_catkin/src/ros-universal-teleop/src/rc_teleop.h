#ifndef RC_TELEOP_H
#define RC_TELEOP_H

#include <ros/ros.h>
#include <mavros_msgs/RCIn.h>
#include <mavros_msgs/ManualControl.h>
#include <mavros_msgs/ParamValue.h>
#include <geometry_msgs/TwistStamped.h>
#include <geometry_msgs/PoseStamped.h>

namespace universal_teleop {
  class RCTeleop {
    public:
      RCTeleop(void);

    private:
			void on_twist(const geometry_msgs::TwistStampedConstPtr& msg);
      void rc_in_event(const mavros_msgs::RCInConstPtr& rcin);
      void manual_control_event(const mavros_msgs::ManualControlConstPtr& manual);
			void on_pose(const geometry_msgs::PoseStampedConstPtr& msg);

			void twist_apply_yaw(geometry_msgs::TwistStamped& twist, const geometry_msgs::PoseStamped& pose);

      std::map<std::string, std::string> rc_switches_map;

			enum class AUX_STATE { HIGH, LOW, NIL };
			std::vector<int64_t> aux_mapping;
			std::vector<AUX_STATE> aux_states;

      ros::NodeHandle n;

			ros::Publisher event_pub, twist_pub, control_pub;
			ros::Subscriber rc_sub, manual_control_sub, twist_sub, pose_sub;

			void pull_parameters(void);
			mavros_msgs::ParamValue get_parameter(const std::string& parameter);

			double max_xy_vel, max_z_vel, max_r_vel;
			double z_dz;
			bool override_enabled;

			geometry_msgs::PoseStamped last_pose;
			geometry_msgs::TwistStamped last_twist;
  };
}

#endif // RC_TELEOP_H

