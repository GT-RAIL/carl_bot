/*!
* \carl_joy_teleop.h
* \brief Allows for control of CARL with a joystick.
*
* carl_joy_teleop creates a ROS node that allows the control of CARL with a joystick.
* This node listens to a /joy topic and sends messages to the /cmd_vel topic.
*
* \author Steven Kordell, WPI - spkordell@wpi.edu
* \date May 23, 2014
*/


#ifndef YOUBOT_JOY_TELEOP_H_
#define YOUBOT_JOY_TELEOP_H_

#include <geometry_msgs/Twist.h>
#include <ros/ros.h>
#include <sensor_msgs/Joy.h>


/*!
* Creates and runs the carl_joy_teleop node.
*
* \param argc argument count that is passed to ros::init
* \param argv arguments that are passed to ros::init
* \return EXIT_SUCCESS if the node runs correctly
*/
int main(int argc, char **argv);

#endif
