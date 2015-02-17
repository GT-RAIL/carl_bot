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
#include <carl_dynamixel/LookAtFrame.h>
#include <carl_dynamixel/LookAtPoint.h>
#include <sensor_msgs/JointState.h>
#include <std_msgs/Float64.h>
#include <tf/transform_listener.h>

#define ASUS_TILT_MIN -1.79
#define ASUS_TILT_MAX 2.59

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

  /**
  * \brief Callback for pointing the servo at a point
  * @param req service request
  * @param res empty service response
  * @return true on success
  */
  bool lookAtPoint(carl_dynamixel::LookAtPoint::Request &req, carl_dynamixel::LookAtPoint::Response &res);

  /**
  * \brief Callback for pointing the servo at a point
  * @param req service request
  * @param res empty service response
  * @return true on success
  */
  bool lookAtFrame(carl_dynamixel::LookAtFrame::Request &req, carl_dynamixel::LookAtFrame::Response &res);

  /**
  * \brief Determine the servo command angle to look at a given point with the Asus servo
  * @param x forward distance from camera
  * @param z height distance from camera
  * @return an angle (rad) to use as a servo command to look at the specified point
  */
  float calculateLookAngle(float x, float z);

  ros::NodeHandle node; /*!< a handle for this ROS node */

  ros::Publisher asusServoControllerPublisher; /*!< position command for the asus servo */
  ros::Publisher creativeServoControllerPublisher; /*!< position command for the creative camera servo */
  ros::Subscriber tiltCommandSubscriber; /*!< subscriber for camera tilt commands */
  ros::Subscriber panCommandSubscriber; /*!< subscriber for camera pan commands */
  ros::Subscriber backJointStateSubscriber; /*!< back (asus) servo joint state subscrbier */
  ros::Subscriber frontJointStateSubscriber; /*!< front (creative) servo joint state subscrbier */

  ros::ServiceServer lookAtPointServer;
  ros::ServiceServer lookAtFrameServer;

  tf::TransformListener tfListener;

  float backServoPos; /*!< the current position of the back servo */
  float frontServoPos; /*!< the current position of the front servo */
};

#endif
