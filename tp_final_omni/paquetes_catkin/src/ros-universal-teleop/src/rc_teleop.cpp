#include <std_msgs/Empty.h>
#include <iostream>
#include <mavros_msgs/RCIn.h>
#include <mavros_msgs/State.h>
#include <mavros_msgs/ParamGet.h>
#include <mavros_msgs/ParamPull.h>
#include <mavros_msgs/ManualControl.h>
#include <universal_teleop/Event.h>
#include <universal_teleop/Control.h>
#include <geometry_msgs/TwistStamped.h>
#include <tf/tf.h>
#include "rc_teleop.h"

using namespace std;
namespace teleop = universal_teleop;

teleop::RCTeleop::RCTeleop(void) : n("~")
{
	n.param("z_dz", z_dz, 0.2);
	override_enabled = false;

	/* pull parameters from autopilot */
	pull_parameters();

  /* get parameter values */
  mavros_msgs::ParamGetResponse get_response;

	/* read AUX switch mapping */
	std::string aux_names[] = { "RC_MAP_AUX1", "RC_MAP_AUX2", "RC_MAP_AUX3" };
	for (int i = 0; i	< 3; i++) {
		aux_mapping.push_back((int)get_parameter(aux_names[i]).integer);
		ROS_INFO_STREAM("AUX" << i + 1 << " is " << (aux_mapping[i] == 0 ? std::string("unmapped") : boost::lexical_cast<std::string>(aux_mapping[i])));
	}

	aux_states.resize(aux_mapping.size(), AUX_STATE::NIL);

	/* read own mapping */
  if (n.hasParam("rc_switches")) n.getParam("rc_switches", rc_switches_map);
	else rc_switches_map = { { "aux1_high", "start" }, { "aux1_low", "stop" } };

	/* read maximum speed values */
	n.param("max_xy_vel", max_xy_vel, get_parameter("MPC_XY_VEL_MAX").real);
	n.param("max_z_vel", max_z_vel, get_parameter("MPC_Z_VEL_MAX").real);
	n.param("max_r_vel", max_r_vel, get_parameter("MPC_MAN_R_MAX").real * (M_PI / 180.0));

	ROS_INFO_STREAM("MAX_XY: " << max_xy_vel << " MAX_Z: " << max_z_vel << " MAX_R: " << max_r_vel);

  /* advertise topics */
  event_pub = n.advertise<teleop::Event>("events", 5);
	control_pub = n.advertise<teleop::Control>("controls", 1);
	twist_pub = n.advertise<geometry_msgs::TwistStamped>("/mavros/setpoint_velocity/cmd_vel", 1);

  /* subscribe topics */
	twist_sub = n.subscribe<geometry_msgs::TwistStamped>("cmd_vel_in", 1, &RCTeleop::on_twist, this);
  rc_sub = n.subscribe<mavros_msgs::RCIn>("/mavros/rc/in", 10, &RCTeleop::rc_in_event, this);
	manual_control_sub = n.subscribe<mavros_msgs::ManualControl>("/mavros/manual_control/control", 1, &RCTeleop::manual_control_event, this);
	pose_sub = n.subscribe<geometry_msgs::PoseStamped>("/mavros/local_position/local", 1, &RCTeleop::on_pose, this);
}

void universal_teleop::RCTeleop::on_twist(const geometry_msgs::TwistStampedConstPtr& msg)
{
	if (last_pose.header.stamp.isZero() || ros::Duration(ros::Time::now() - last_pose.header.stamp).toSec() > 0.3) { ROS_WARN("No valid pose, not sending twist"); return; }

	geometry_msgs::TwistStamped new_twist(*msg);
	twist_apply_yaw(new_twist, last_pose);
	twist_pub.publish(new_twist);
}

void teleop::RCTeleop::rc_in_event(const mavros_msgs::RCInConstPtr& rcin)
{
  if (rcin->channels.empty()) { ROS_WARN("Empty RC message"); return; }

	for (int i = 0; i < aux_mapping.size(); i++)
	{
		if (aux_mapping[i] == 0) continue; // ignore unmapped switches

		int16_t channel_value	= rcin->channels[aux_mapping[i] - 1];
		AUX_STATE new_state = AUX_STATE::NIL;
		if (channel_value > 1700) new_state = AUX_STATE::HIGH;
		else if (channel_value < 1300) new_state = AUX_STATE::LOW;

		if (new_state != AUX_STATE::NIL && aux_states[i] != new_state)
		{
			ROS_INFO_STREAM("AUX" << i + 1 << ": " << (new_state == AUX_STATE::HIGH ? "HIGH" : "LOW"));
			aux_states[i] = new_state;

			universal_teleop::Event ev;
			ev.header = rcin->header;

			std::string aux_name = std::string("aux") + boost::lexical_cast<std::string>(i + 1);
			if (rc_switches_map.find(aux_name) != rc_switches_map.end()) {
				ev.event = rc_switches_map[aux_name];
				ev.state = (new_state == AUX_STATE::HIGH);
			}
			else if (new_state == AUX_STATE::HIGH && rc_switches_map.find(aux_name + "_high") != rc_switches_map.end()) {
				ev.event = rc_switches_map[aux_name + "_high"];
				ev.state = 1;
			}
			else if (new_state == AUX_STATE::LOW && rc_switches_map.find(aux_name + "_low") != rc_switches_map.end()) {
				ev.event = rc_switches_map[aux_name + "_low"];
				ev.state = 1;
			}

			if (!ev.event.empty())
				event_pub.publish(ev);
		}
	}
}

void universal_teleop::RCTeleop::manual_control_event(const mavros_msgs::ManualControlConstPtr& manual)
{
	/* handle special case of throttle stick */
	float manual_z = (manual->z - 0.5) * 2.0; // throttle stick goes [0,1] and I want [-1,1]
	manual_z = std::abs(manual_z) < z_dz ? 0 : ((manual_z + (manual_z < 0 ? z_dz : -z_dz)) / (1 - z_dz)); // apply DZ and rescale

	/* read mode switch position to test for offboard mode */
	bool offboard_requested = ((manual->buttons & (1 << 11)) != 0); // 0b11 = OFF, 0b01 = ON; offboard is bit 10-11

	/* send override event if state changed */
	if (override_enabled != offboard_requested)
	{
		override_enabled = offboard_requested;

		universal_teleop::Event ev;
		ev.header.stamp = manual->header.stamp;
		ev.event = "override";
		ev.state = override_enabled;
		event_pub.publish(ev);
	}

	/* send controls */
	universal_teleop::Control control_msg;
	control_msg.header = manual->header;
	control_msg.controls = { "pitch", "roll", "vertical", "yaw" };
	control_msg.values = { manual->x, -manual->y, manual_z, -manual->r };
	control_pub.publish(control_msg);
}

void universal_teleop::RCTeleop::on_pose(const geometry_msgs::PoseStampedConstPtr& msg)
{
	last_pose = *msg;
	last_pose.header.stamp = ros::Time::now();
}

/** Rotates a twist message to local frame, since MAVROS currently assumes this */
void universal_teleop::RCTeleop::twist_apply_yaw(geometry_msgs::TwistStamped& twist, const geometry_msgs::PoseStamped& pose)
{
	tf::Vector3 linear_speed;
	tf::vector3MsgToTF(twist.twist.linear, linear_speed);

	tf::Quaternion Q;
	tf::quaternionMsgToTF(pose.pose.orientation, Q);
	tf::Transform rot(tf::createQuaternionFromYaw(tf::getYaw(Q)));
	linear_speed = rot * linear_speed;

	tf::vector3TFToMsg(linear_speed, twist.twist.linear);
}

void universal_teleop::RCTeleop::pull_parameters(void)
{
	mavros_msgs::ParamPullRequest pull_request;
	mavros_msgs::ParamPullResponse pull_response;
	if (!ros::service::waitForService("/mavros/param/pull")) throw std::runtime_error("Could not pull MAVROS parameters");

	ROS_INFO_STREAM("Requesting parameters...");
	if (!ros::service::call("/mavros/param/pull", pull_request, pull_response)) throw std::runtime_error("Failed to pull MAVROS parameters");
}

mavros_msgs::ParamValue universal_teleop::RCTeleop::get_parameter(const string& parameter)
{
	mavros_msgs::ParamGetRequest get_request;
	mavros_msgs::ParamGetResponse get_response;
	get_request.param_id = parameter;
	if (!ros::service::waitForService("/mavros/param/get")) throw std::runtime_error("Could not pull MAVROS parameters");

	ROS_INFO_STREAM("Requesting " << parameter << " parameter");
	if (!ros::service::call("/mavros/param/get", get_request, get_response)) throw std::runtime_error("Failed to get parameter");

	return get_response.value;
}
