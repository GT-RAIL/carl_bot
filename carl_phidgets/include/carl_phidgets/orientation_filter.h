/*!
 * \orientation_filter.h
 * \brief Calculates the roll and pitch of the base and the pitch of the sensor tower from CARL's IMUs.
 *
 * orientation_filter creates a ROS node that measures orientations for the base and sensor tower by combining and
 * filtering accelerometer and gyro data.
 *
 * \author David Kent - davidkent@wpi.edu
 * \date January 13, 2015
 */

#ifndef ORIENTATION_FILTER_H_
#define ORIENTATION_FILTER_H_

#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/JointState.h>

//Arm Parameters
/*!
 * \def G
 *
 * Gravity constant
 */
#define G 9.81

class OrientationFilter
{
public:
  /*!
   * Constructor
   */
  OrientationFilter();

private:
  /*!
   * Base IMU callback.
   *
   * \param data data from the base IMU
   */
  void baseImuCallback(const sensor_msgs::Imu::ConstPtr& data);

  /*!
   * Top IMU callback.
   *
   * \param data data from the top IMU
   */
  void topImuCallback(const sensor_msgs::Imu::ConstPtr& data);

  ros::NodeHandle n;

  ros::Publisher frameJointStatePublisher; /*!< joint state publisher for various parts of CARL's frame */
  ros::Subscriber baseImuSubscriber; /*!< Subscriber for data from the base IMU */
  ros::Subscriber topImuSubscriber; /*!< Subscriber for data from the top IMU */

  sensor_msgs::JointState jointStates;
  bool baseOrientationInitialized;
  ros::Time prevUpdateTimeTop;
  ros::Time prevUpdateTime;
  float PPrevTop;
  float PPrev[2];
};

#endif
