/*!
 * \carl_joy_teleop.cpp
 * \brief Allows for control of CARL with a joystick.
 *
 * carl_joy_teleop creates a ROS node that allows the control of CARL with a joystick. This node listens to a /joy topic
 * and sends messages to the /cmd_vel topic.
 *
 * \author Russell Toris, WPI - rctoris@wpi.edu *
 * \author Steven Kordell, WPI - spkordell@wpi.edu
 * \date June 10, 2014
 */

#include <geometry_msgs/Twist.h>
#include <ros/ros.h>
#include <sensor_msgs/Joy.h>
#include <carl_teleop/carl_joy_teleop.h>

using namespace std;

carl_joy_teleop::carl_joy_teleop()
{
  // create the ROS topics
  cmd_vel = node.advertise<geometry_msgs::Twist>("cmd_vel", 10);
  joy_sub = node.subscribe<sensor_msgs::Joy>("joy", 10, &carl_joy_teleop::joy_cback, this);

  //read in throttle value
  double temp;
  if (node.getParam("/carl_joy_teleop/linear_throttle_factor", temp))
    linear_throttle_factor = (float)temp;
  else
    linear_throttle_factor = 1.0;
  if (node.getParam("/carl_joy_teleop/angular_throttle_factor", temp))
    angular_throttle_factor = (float)temp;
  else
    angular_throttle_factor = 1.0;

  ROS_INFO("Carl Joystick Teleop Started");
}

void carl_joy_teleop::joy_cback(const sensor_msgs::Joy::ConstPtr& joy)
{
  // create the twist message
  geometry_msgs::Twist twist;

  twist.linear.y = 0;
  twist.linear.z = 0;
  twist.angular.x = 0;
  twist.angular.y = 0;

  if (joy->buttons.at(4) == 1)
  {
    // left joystick controls the linear and angular movement
    twist.linear.x = joy->axes.at(1) * MAX_TRANS_VEL * linear_throttle_factor;
    twist.angular.z = -joy->axes.at(2) * MAX_ANG_VEL * angular_throttle_factor;
  }
  else
  {
    twist.linear.x = 0;
    twist.angular.z = 0;
  }

  //boost button
  if (joy->buttons.at(5) == 1)
  {
    twist.linear.x *= 2;
    twist.angular.z *= 2;
  }

  // send the twist command
  cmd_vel.publish(twist);
}

int main(int argc, char **argv)
{
  // initialize ROS and the node
  ros::init(argc, argv, "carl_joy_teleop");

  //initialize the joystick controller
  carl_joy_teleop controller;

  ros::spin();
}
