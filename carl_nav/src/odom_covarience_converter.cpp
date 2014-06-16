/*!
 * \odom_covarience_converter.cpp
 * \brief Adds covarience matrix to odometry message
 *
 * odom_covarience_converter adds a covarience matrix to odometry messages so they are compatible with robot_pose_efk.
 *
 * \author Steven Kordell, WPI - spkordell@wpi.edu
 * \date June 16, 2014
 */

#include <nav_msgs/Odometry.h>
#include <ros/ros.h>
#include <carl_nav/odom_covarience_converter.h>

using namespace std;

odom_covarience_converter::odom_covarience_converter()
{
  // create the ROS topics
  odom_in = node.subscribe<nav_msgs::Odometry>("odom", 10, &odom_covarience_converter::convert_cback, this);
  odom_out = node.advertise<nav_msgs::Odometry>("covarience_odom", 10);

  ROS_INFO("Odometry Covarience Converter Started");
}

void odom_covarience_converter::convert_cback(const nav_msgs::Odometry::ConstPtr& odom)
{

  //odom->pose.pose; //TODO :set this
  //odom->twist->covarience; //TODO : set this

  //odom_out.publish(odom);
}

int main(int argc, char **argv)
{
  // initialize ROS and the node
  ros::init(argc, argv, "odom_covarience_converter");

  // initialize the converter
  odom_covarience_converter converter;

  ros::spin();

  return EXIT_SUCCESS;
}
