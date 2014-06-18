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

  ros::NodeHandle node("~"); /*!< a handle for this ROS node */

  // create the ROS topics
  odom_in = node.subscribe<nav_msgs::Odometry>("odom_in", 10, &odom_covariance_converter::convert_cback, this);
  odom_out = node.advertise<nav_msgs::Odometry>("odom_out", 10);

  // read in covariance parameters
  //const double defaultCov = 1e9;
  node.getParam("cov_x", cov_x);
  node.getParam("cov_y", cov_y);
  node.getParam("cov_z", cov_z);
  node.getParam("cov_rx", cov_rx);
  node.getParam("cov_ry", cov_ry);
  node.getParam("cov_rz", cov_rz);

  ROS_INFO("Odometry covariance Converter Started");

  ROS_INFO("Covarience parameters [%f, %f, %f, %f, %f, %f]", cov_x, cov_y, cov_z, cov_rx, cov_ry, cov_rz);
}

void odom_covariance_converter::convert_cback(const nav_msgs::Odometry::ConstPtr& odom)
{
  nav_msgs::Odometry odometry = *odom;

  odometry.pose.covariance =  boost::assign::list_of (cov_x)  (0)  (0)  (0)  (0)  (0)
                                                       (0) (cov_y) (0)  (0)  (0)  (0)
                                                       (0)  (0) (cov_z) (0)  (0)  (0)
                                                       (0)  (0)  (0) (cov_rx) (0)  (0)
                                                       (0)  (0)  (0)  (0) (cov_ry) (0)
                                                       (0)  (0)  (0)  (0)  (0) (cov_rz);

  /*
  odometry.twist.covariance =  boost::assign::list_of (1e-6) (0)  (0)  (0)  (0)  (0)
                                                       (0) (1e-6) (0)  (0)  (0)  (0)
                                                       (0)  (0) (1e-6) (0)  (0)  (0)
                                                       (0)  (0)  (0) (1e-6) (0)  (0)
                                                       (0)  (0)  (0)  (0) (1e-6) (0)
                                                       (0)  (0)  (0)  (0)  (0) (1e-6);
  */
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
