/*!
 * \back_joints_node.h
 * \breief Allows the stopping of the segway base without losing power to peripherals.
 *
 * carl_estop_node creates a ROS node that allows the stopping of the robot after a specified amount of time. This node
 * listens to a /carl_estop topic and sends messages to the /move_base actionlib to stop the current execution of a
 * goal.
 *
 * \author Russell Toris, WPI - russell.toris@gmail.com
 * \author Chris Dunkers, WPI - cmdunkers@wpi.edu
 * \date July 24, 2014
 */

#ifndef BACK_JOINTS_NODE_H_
#define BACK_JOINTS_NODE_H_

#include <ros/ros.h>
#include <sensor_msgs/JointState.h>
#include <dynamixel_msgs/MotorStateList.h>
#include <dynamixel_msgs/MotorState.h>
#include <math.h>

class back_joints
{
public:
  back_joints();
  /*!
   * estop function to check the time difference to see if any goal position in move_base should be cancelled.
   *
   * \param msg the empty message
   */
  void joints(const dynamixel_msgs::MotorStateList::ConstPtr& state);

private:
  // main node handle
  ros::NodeHandle node;

  // the ros subscriber and publisher
  ros::Subscriber asus_sub;
  ros::Publisher joint_states_pub;

  // map to store the ids and the link names of each servo connected
  std::map<int, std::string> back_servos;
};

#endif
