/*!
* \carl_joy_teleop.cpp
* \brief Allows for control of CARL with a joystick.
*
* carl_joy_teleop creates a ROS node that allows the control of CARL with a joystick.
* This node listens to a /joy topic and sends messages to the /cmd_vel topic.
*
* \author Steven Kordell, WPI - spkordell@wpi.edu
* \date May 23, 2014
*/



#include <geometry_msgs/Twist.h>
#include <ros/ros.h>
#include <sensor_msgs/Joy.h>
#include <carl_teleop/carl_joy_teleop.h>

ros::Time T;


int main(int argc, char **argv) {
  // initialize ROS and the node
  ros::init(argc, argv, "carl_joy_teleop");

  // initialize the joystick controller
  //carl_joy_teleop controller;

  // continue until a ctrl-c has occurred
  while(ros::ok()) {
	  //controller.joy_check();

	  ros::spinOnce();
  }
  //ros::spin();
}
