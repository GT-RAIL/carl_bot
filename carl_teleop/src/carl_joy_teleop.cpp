/*!
 * \carl_joy_teleop.cpp
 * \brief Allows for control of CARL with a joystick.
 *
 * carl_joy_teleop creates a ROS node that allows the control of CARL with a joystick. This node listens to a /joy topic
 * and sends messages to the /cmd_vel topic.
 *
 * \author Russell Toris, WPI - rctoris@wpi.edu *
 * \author Steven Kordell, WPI - spkordell@wpi.edu
 * \date June 10, 2014
 */

#include <geometry_msgs/Twist.h>
#include <ros/ros.h>
#include <sensor_msgs/Joy.h>
#include <carl_teleop/carl_joy_teleop.h>

using namespace std;

carl_joy_teleop::carl_joy_teleop()
{
  // create the ROS topics
  cmd_vel = node.advertise<geometry_msgs::Twist>("cmd_vel", 10);
  joy_sub = node.subscribe<sensor_msgs::Joy>("joy", 10, &carl_joy_teleop::joy_cback, this);

  // read in throttle values
  double temp;
  if (node.getParam("/carl_joy_teleop/linear_throttle_factor", temp))
    linear_throttle_factor = (float)temp;
  else
    linear_throttle_factor = 1.0;
  if (node.getParam("/carl_joy_teleop/angular_throttle_factor", temp))
    angular_throttle_factor = (float)temp;
  else
    angular_throttle_factor = 1.0;

  //initialize state of deadman switch
  deadmanPressed = false;

  // Connect to the move_base action server
  actionClient = new ActionClient("move_base", true); // create a thread to handle subscriptions.

  ROS_INFO("Carl Joystick Teleop Started");
}

void carl_joy_teleop::joy_cback(const sensor_msgs::Joy::ConstPtr& joy)
{
  // create the twist message
  geometry_msgs::Twist twist;

  twist.linear.y = 0;
  twist.linear.z = 0;
  twist.angular.x = 0;
  twist.angular.y = 0;

  //Latch the deadman switch state
  bool oldDeadmanPressed = deadmanPressed;

  //If button 1 on the controller is pressed, cancel the navigation goal planning
  if (joy->buttons.at(0) == 1)
  {
    twist.linear.x = 0;
    twist.angular.z = 0;
    cmd_vel.publish(twist);
    actionClient->waitForServer();
    actionClient->cancelAllGoals();
  }

  if (joy->buttons.at(4) == 1)
  {
    // left joystick controls the linear and angular movement
    twist.linear.x = joy->axes.at(1) * MAX_TRANS_VEL * linear_throttle_factor;
    twist.angular.z = joy->axes.at(2) * MAX_ANG_VEL * angular_throttle_factor;
    deadmanPressed = true;
  }
  else
  {
    twist.linear.x = 0;
    twist.angular.z = 0;
    deadmanPressed = false;
  }

  //boost button
  if (joy->buttons.at(5) == 1)
  {
    twist.linear.x *= 3;
    twist.angular.z *= 3;
  }

  // send the twist command
  if (deadmanPressed || oldDeadmanPressed) {
    cmd_vel.publish(twist);
  }
}

int main(int argc, char **argv)
{
  // initialize ROS and the node
  ros::init(argc, argv, "carl_joy_teleop");

  // initialize the joystick controller
  carl_joy_teleop controller;

  ros::spin();

  return EXIT_SUCCESS;
}
