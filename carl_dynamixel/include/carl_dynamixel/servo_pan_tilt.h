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
   * Servo joint state callback.
   *
   * \param msg servo joint states
   */
  void jointStateCallback(const sensor_msgs::JointState::ConstPtr& msg);

  /*!
   * Servo tilt command callback
   * @param msg message denoting the desired tilt velocity (rad/s)
   */
  void tiltCallback(const std_msgs::Float64::ConstPtr& msg);

  ros::NodeHandle node; /*!< a handle for this ROS node */

  ros::Publisher asusServoControllerPublisher; /*!< position command for the asus servo */
  ros::Subscriber tiltCommandSubscriber; /*!< subscriber for camera tilt commands */
  ros::Subscriber jointStateSubscriber; /*!< servo joint state subscrbier */

  float servoPos; /*!< the current position of the servo */
};

#endif
