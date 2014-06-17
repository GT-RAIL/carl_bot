/*!
 * \odom_covariance_converter.cpp
 * \brief Adds covariance matrix to odometry message
 *
 * odom_covariance_converter adds a covariance matrix to odometry messages so they are compatible with robot_pose_efk.
 *
 * \author Steven Kordell, WPI - spkordell@wpi.edu
 * \date June 16, 2014
 */

#include <nav_msgs/Odometry.h>
#include <ros/ros.h>
#include <carl_nav/odom_covariance_converter.h>
#include <boost/assign/list_of.hpp>

using namespace std;

odom_covariance_converter::odom_covariance_converter()
{
  // create the ROS topics
  string odom_in_topic;
  string odom_out_topic;
  node.param("in_topic", odom_in_topic, odom_in_topic);
  node.param("out_topic", odom_out_topic, odom_out_topic);

  odom_in = node.subscribe<nav_msgs::Odometry>(odom_in_topic, 10, &odom_covariance_converter::convert_cback, this);
  odom_out = node.advertise<nav_msgs::Odometry>(odom_out_topic, 10);

  ROS_INFO("Odometry covariance Converter Started");
}

void odom_covariance_converter::convert_cback(const nav_msgs::Odometry::ConstPtr& odom)
{
  nav_msgs::Odometry odometry = *odom;

  odometry.pose.covariance =  boost::assign::list_of(1e-3) (0)  (0)  (0)  (0)  (0)
                                                  (0) (1e-3) (0)  (0)  (0)  (0)
                                                  (0)   (0) (1e6) (0)  (0)  (0)
                                                  (0)   (0)  (0) (1e6) (0)  (0)
                                                  (0)   (0)  (0)  (0) (1e6) (0)
                                                  (0)   (0)  (0)  (0)  (0) (1e3);

  odometry.twist.covariance =  boost::assign::list_of(1e-3) (0)  (0)  (0)  (0)  (0)
                                                  (0) (1e-3) (0)  (0)  (0)  (0)
                                                  (0)   (0) (1e6) (0)  (0)  (0)
                                                  (0)   (0)  (0) (1e6) (0)  (0)
                                                  (0)   (0)  (0)  (0) (1e6) (0)
                                                  (0)   (0)  (0)  (0)  (0) (1e3);

  odom_out.publish(odometry);
}

int main(int argc, char **argv)
{
  // initialize ROS and the node
  ros::init(argc, argv, "odom_covariance_converter");

  // initialize the converter
  odom_covariance_converter converter;

  ros::spin();

  return EXIT_SUCCESS;
}
