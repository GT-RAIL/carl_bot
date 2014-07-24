/*!
 * \carl_joy_teleop.cpp
 * \brief Allows for control of CARL with a joystick.
 *
 * carl_joy_teleop creates a ROS node that allows the control of CARL with a joystick. This node listens to a /joy topic
 * and sends messages to the /cmd_vel topic.
 *
 * \author Russell Toris, WPI - rctoris@wpi.edu
 * \author Steven Kordell, WPI - spkordell@wpi.edu
 * \date July 24, 2014
 */

#include <carl_teleop/carl_joy_teleop.h>

carl_joy_teleop::carl_joy_teleop()
{
  // a private handle for this ROS node (allows retrieval of relative parameters)
  ros::NodeHandle private_nh("~");

  // create the ROS topics
  cmd_vel = node.advertise<geometry_msgs::Twist>("cmd_vel", 10);
  joy_sub = node.subscribe<sensor_msgs::Joy>("joy", 10, &carl_joy_teleop::joy_cback, this);

  // read in throttle values
  private_nh.param<double>("linear_throttle_factor", linear_throttle_factor, 1.0);
  private_nh.param<double>("angular_throttle_factor", angular_throttle_factor, 1.0);

  // initialize state of deadman switch
  deadman = false;

  ROS_INFO("Carl Joystick Teleop Started");
}

void carl_joy_teleop::joy_cback(const sensor_msgs::Joy::ConstPtr& joy)
{
  // create the twist message
  geometry_msgs::Twist twist;

  // save the deadman switch state
  bool was_pressed = deadman;

  if (joy->buttons.at(4) == 1)
  {
    // left joystick controls the linear and angular movement
    twist.linear.x = joy->axes.at(1) * MAX_TRANS_VEL * linear_throttle_factor * NON_BOOST_THROTTLE;
    twist.angular.z = joy->axes.at(2) * MAX_ANG_VEL * angular_throttle_factor * NON_BOOST_THROTTLE;
    deadman = true;
  }
  else
    deadman = false;

  // boost button
  if (joy->buttons.at(5) == 1)
  {
    twist.linear.x = MAX_TRANS_VEL * linear_throttle_factor;
    twist.angular.z = MAX_ANG_VEL * angular_throttle_factor;
  }

  // send the twist command
  if (deadman || was_pressed)
    cmd_vel.publish(twist);
}

int main(int argc, char **argv)
{
  // initialize ROS and the node
  ros::init(argc, argv, "carl_joy_teleop");

  // initialize the joystick controller
  carl_joy_teleop controller;

  // wait for ctrl-c
  ros::spin();

  return EXIT_SUCCESS;
}
