#ifndef __TELEOP_H__
#define __TELEOP_H__

#include <ros/ros.h>
#include <sensor_msgs/Joy.h>
#include <geometry_msgs/Twist.h>
#include <keyboard/Key.h>
#include <universal_teleop/Event.h>

namespace universal_teleop {
  class Teleop {
    public:
      Teleop(void);
      void control(void);

    private:

      void velocity_command(const geometry_msgs::Twist::ConstPtr& vel);

      void joystick_event(const sensor_msgs::Joy::ConstPtr& joy);
      void keyboard_up_event(const keyboard::Key::ConstPtr& key);
      void keyboard_down_event(const keyboard::Key::ConstPtr& key);

      void process_event(const universal_teleop::Event& e);

      ros::NodeHandle n;

      // Optional subscription to an axternal controller,
      // which has lower priority than the override.
      ros::Subscriber vel_sub;

      ros::Subscriber joy_sub, keyup_sub, keydown_sub;
      ros::Publisher pub_vel, pub_event, pub_control;
      ros::Publisher pub_takeoff, pub_land, pub_emergency;

      sensor_msgs::Joy last_joy_msg;
      std::map<int, std::string> joy_button_map;
      std::map<int, std::string> joy_axis_map;
      std::map<uint16_t, std::string> key_map;
      std::map<uint16_t, std::string> key_axes_map;
      std::map<std::string, int> key_axes_state;
      
      std::map<std::string, float> axis_scales;

      std::map<std::string, int> joy_axes;
      std::map<std::string, float> joy_deadzones;

      bool key_override_enabled, joy_override_enabled, send_velocity;
  };
}

#endif
