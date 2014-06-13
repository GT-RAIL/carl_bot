/*!
 * \carl_joy_teleop.h
 * \brief Allows for control of CARL with a joystick.
 *
 * carl_joy_teleop creates a ROS node that allows the control of CARL with a joystick. This node listens to a /joy topic
 * and sends messages to the /cmd_vel topic.
 *
 * \author Russell Toris, WPI - rctoris@wpi.edu *
 * \author Steven Kordell, WPI - spkordell@wpi.edu
 * \date June 10, 2014
 */

#ifndef CARL_JOY_TELEOP_H_
#define CARL_JOY_TELEOP_H_

#include <geometry_msgs/Twist.h>
#include <ros/ros.h>
#include <sensor_msgs/Joy.h>

/*!
 * \def MAX_TRANS_VEL
 *
 * The maximum translational velocity.
 */
#define MAX_TRANS_VEL .8

/*!
 * \def MAX_ANG_VEL
 *
 * The maximum angular velocity.
 */
#define MAX_ANG_VEL 1.2

/*!
 * \class carl_joy_teleop
 * \brief Allows for control of CARL with a joystick.
 *
 * carl_joy_teleop creates a ROS node that allows the control of CARL with a joystick. This node listens to a /joy topic
 * and sends messages to the /cmd_vel topic.
 */
class carl_joy_teleop
{
public:
  /*!
   * Creates a carl_joy_teleop object that can be used control carl with a joystick. ROS nodes, services, and publishers
   * are created and maintained within this object.
   */
  carl_joy_teleop();

private:
  /*!
   * Joy topic callback function.
   *
   * \param joy the message for the joy topic
   */
  void joy_cback(const sensor_msgs::Joy::ConstPtr& joy);

  ros::NodeHandle node; /*!< a handle for this ROS node */

  ros::Publisher cmd_vel; /*!< the cmd_vel topic */
  ros::Subscriber joy_sub; /*!< the joy topic */

  float linear_throttle_factor; /*!< factor for reducing the maximum linear speed */
  float angular_throttle_factor; /*!< factor for reducing the maximum angular speed */

  bool deadmanPressed; /*!< save state of deadman switch */
};

/*!
 * Creates and runs the carl_joy_teleop node.
 *
 * \param argc argument count that is passed to ros::init
 * \param argv arguments that are passed to ros::init
 * \return EXIT_SUCCESS if the node runs correctly
 */
int main(int argc, char **argv);

#endif
