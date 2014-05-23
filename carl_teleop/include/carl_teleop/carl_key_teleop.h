/*
* \carl_key_teleop.cpp
* \brief Allows for control of CARL with a keyboard.
*
* carl_joy_teleop creates a ROS node that allows the control of CARL with a keyboard.
* This node listens to a /joy topic and sends messages to the /cmd_vel topic.
*
* \author Steven Kordell, WPI - spkordell@wpi.edu
* \date May 23, 2014
*/


#ifndef CARL_JOY_TELEOP_H_
#define CARL_JOY_TELEOP_H_

#include <geometry_msgs/Twist.h>
#include <ros/ros.h>
#include <sensor_msgs/Joy.h>

#define MAX_TRANS_VEL .8
#define MAX_ANG_VEL 1.2

class carl_key_teleop {
public:
  carl_key_teleop();
  void keyLoop();
  void watchdog();

private:
  ros::NodeHandle nh_,ph_;
  double linear_, angular_;
  ros::Time first_publish_;
  ros::Time last_publish_;
  double l_scale_, a_scale_;
  ros::Publisher vel_pub_;
  void publish(double, double);
  boost::mutex publish_mutex_;
};


/*!
* Creates and runs the carl_key_teleop node.
*
* \param argc argument count that is passed to ros::init
* \param argv arguments that are passed to ros::init
* \return EXIT_SUCCESS if the node runs correctly
*/
int main(int argc, char **argv);

#endif
