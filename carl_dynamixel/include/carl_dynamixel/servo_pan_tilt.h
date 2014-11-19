/*!
 * \servo_pan_tilt.h
 * \brief Controls panning and tilting of CARL's cameras.
 *
 * servo_pan_tilt creates a ROS node that allows pan and tilt control of
 * the servos that CARL's cameras are mounted on.
 *
 * \author David Kent, WPI - davidkent@wpi.edu
 * \date August 5, 2014
 */

#ifndef SERVO_PAN_TILT_H_
#define SERVO_PAN_TILT_H_

#include <ros/ros.h>
#include <sensor_msgs/JointState.h>
#include <std_msgs/Float64.h>

/*!
 * \class servoPanTilt
 * \brief Controls panning and tilting of CARL's cameras.
 *
 * servoPanTilt creates a ROS node that allows pan and tilt control of
 * the servos that CARL's cameras are mounted on.
 */
class servoPanTilt
{
public:
  /*!
   * \brief Constructor
   */
  servoPanTilt();

private:
  /*!
   * Back servo joint state callback.
   *
   * \param msg servo joint states
   */
  void backJointStateCallback(const sensor_msgs::JointState::ConstPtr& msg);
  
  /*!
   * Front servo joint state callback.
   *
   * \param msg servo joint states
   */
  void frontJointStateCallback(const sensor_msgs::JointState::ConstPtr& msg);

  /*!
   * Servo tilt command callback
   * @param msg message denoting the desired tilt velocity (rad/s)
   */
  void tiltCallback(const std_msgs::Float64::ConstPtr& msg);
  
  /*!
   * Servo pan command callback
   * @param msg message denoting the desired pan velocity (rad/s)
   */
  void panCallback(const std_msgs::Float64::ConstPtr& msg);

  ros::NodeHandle node; /*!< a handle for this ROS node */

  ros::Publisher asusServoControllerPublisher; /*!< position command for the asus servo */
  ros::Publisher creativeServoControllerPublisher; /*!< position command for the creative camera servo */
  ros::Subscriber tiltCommandSubscriber; /*!< subscriber for camera tilt commands */
  ros::Subscriber panCommandSubscriber; /*!< subscriber for camera pan commands */
  ros::Subscriber backJointStateSubscriber; /*!< back (asus) servo joint state subscrbier */
  ros::Subscriber frontJointStateSubscriber; /*!< front (creative) servo joint state subscrbier */

  float backServoPos; /*!< the current position of the back servo */
  float frontServoPos; /*!< the current position of the front servo */
};

#endif
