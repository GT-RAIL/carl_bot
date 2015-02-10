/*!
 * \file current_logger.h
 * \brief Output of arm joint currents to a csv file
 *
 * \author David Kent, WPI - davidkent@wpi.edu
 * \date February 10, 2015
 */

#ifndef CURRENT_LOGGER_H
#define CURRENT_LOGGER_H

#include <iostream>
#include <fstream>
#include <ros/ros.h>
#include <sensor_msgs/JointState.h>

class CurrentLogger
{
public:
  /**
  * \brief Constructor
  */
  CurrentLogger();

private:
  ros::NodeHandle n;

  ros::Subscriber armJointStateSubscriber;

  std::string filename;

  /**
  * \brief get samples if calibration is active
  *
  * @param msg marker pose data
  */
  void armJointStateCallback(const sensor_msgs::JointState::ConstPtr& msg);
};

int main(int argc, char **argv);

#endif
