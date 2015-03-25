/*!
 * \carl_joy_teleop.h
 * \brief Allows for control of CARL with a joystick.
 *
 * carl_joy_teleop creates a ROS node that allows the control of CARL with a joystick. 
 * This node listens to a /joy topic and sends messages to the /cmd_vel topic for 
 * the base and cartesian_cmd for the arm.
 *
 * \author David Kent, WPI - davidkent@wpi.edu
 * \author Russell Toris, WPI - rctoris@wpi.edu
 * \author Steven Kordell, WPI - spkordell@wpi.edu
 * \date July 24, 2014
 */

#ifndef CARL_JOY_TELEOP_H_
#define CARL_JOY_TELEOP_H_

#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>
#include <carl_moveit/ArmAction.h>
#include <sensor_msgs/Joy.h>
#include <std_msgs/Float64.h>
#include <std_srvs/Empty.h>
#include <wpi_jaco_msgs/AngularCommand.h>
#include <wpi_jaco_msgs/CartesianCommand.h>
#include <wpi_jaco_msgs/EStop.h>

//Control modes
#define ARM_CONTROL 0 
#define FINGER_CONTROL 1
#define BASE_CONTROL 2
#define SENSOR_CONTROL 3

//Joystick types
#define ANALOG 0 //analog triggers
#define DIGITAL 1 //digital triggers

//Arm Parameters
/*!
 * \def MAX_TRANS_VEL_ARM
 *
 * The maximum translational velocity.
 */
#define MAX_TRANS_VEL_ARM .175

/*!
 * \def MAX_ANG_VEL_ARM
 *
 * The maximum angular velocity.
 */
#define MAX_ANG_VEL_ARM 1.047

/*!
 * \def MAX_FINGER_VEL
 * The maximum velocity for a finger.
 */
#define MAX_FINGER_VEL 30

//Base Parameters
/*!
 * \def MAX_TRANS_VEL
 *
 * The maximum translational velocity.
 */
#define MAX_TRANS_VEL_BASE 1.0

/*!
 * \def MAX_ANG_VEL
 *
 * The maximum angular velocity.
 */
#define MAX_ANG_VEL_BASE 1.0

/*!
 * \def NON_BOOST_THROTTLE
 *
 * The throttle factor for a non-boost command.
 */
#define NON_BOOST_THROTTLE 0.8

/*!
 * \class carl_joy_teleop
 * \brief Allows for control of CARL with a joystick.
 *
 * carl_joy_teleop creates a ROS node that allows the control of CARL with a joystick. 
 * This node listens to a /joy topic and sends messages to the /cmd_vel topic.
 */
class carl_joy_teleop
{
public:
  /*!
   * Creates a carl_joy_teleop object that can be used control carl with a joystick. ROS nodes, services, and publishers
   * are created and maintained within this object.
   */
  carl_joy_teleop();

  /*!
   * Periodically publish velocity message to the arm controller
   */
  void publish_velocity();

private:
  /*!
   * Joy topic callback function.
   *
   * \param joy the message for the joy topic
   */
  void joy_cback(const sensor_msgs::Joy::ConstPtr& joy);

  void displayHelp(int menuNumber);

  ros::NodeHandle node; /*!< a handle for this ROS node */

  ros::Publisher cmd_vel; /*!< the base cmd_vel topic */
  ros::Publisher angular_cmd; /*!< angular arm command topic */
  ros::Publisher cartesian_cmd; /*!< cartesian arm command topic */
  ros::Publisher cartesian_cmd2; /*!< topic for testing alternate Cartesian controllers */
  ros::Publisher asus_servo_tilt_cmd; /*< velocity command to tilt the asus servo */
  ros::Publisher creative_servo_pan_cmd; /*< velocity command to pan the creative servo */
  ros::Subscriber joy_sub; /*!< the joy topic */

  ros::ServiceClient segment_client; /*!< the asus point cloud segmentation client */
  ros::ServiceClient eStopClient; /*!< arm software emergency stop service client */

  actionlib::SimpleActionClient<carl_moveit::ArmAction> acArm; /*!< home and retract arm action client */

  geometry_msgs::Twist twist; /*!< base movement command */
  wpi_jaco_msgs::AngularCommand fingerCmd; /*!< finger movement command */
  wpi_jaco_msgs::CartesianCommand cartesianCmd; /*!< cartesian movement command */
  geometry_msgs::Twist cartesianCmd2;

  int mode; /*!< the control mode */
  int controllerType; /*!< the type of joystick controller */
  int rightBumperPrev; /*!< previous state of the right bumper, used for detecting state changes */
  int leftBumperPrev; /*!< previous state of the left bumper, used for detecting state changes */
  int rightStickPrev; /*!< previous state of right stick button press, used for detecting state changes */
  double linear_throttle_factor_base; /*!< factor for reducing the base maximum linear speed */
  double angular_throttle_factor_base; /*!< factor for reducing the base maximum angular speed */
  double linear_throttle_factor_arm; /*!< factor for reducing the arm linear speed */
  double angular_throttle_factor_arm; /*!< factor for reducing the arm angular speed */
  double finger_throttle_factor; /*!< factor for reducing the finger speed */
  bool stopMessageSentArm; /*!< flag to prevent the arm stop command from being sent repeatedly when the controller is in the neutral position */
  bool stopMessageSentFinger; /*!< flag to prevent the finger stop command from being sent repeatedly when the controller is in the neutral position */
  bool initLeftTrigger; /*!< flag for whether the left trigger is initialized */
  bool initRightTrigger; /*!< flag for whether the right trigger is initialized */
  bool calibrated; /*!< flag for whether the controller is calibrated, this only affects controllers with analog triggers */
  bool EStopEnabled; /*!< software emergency stop for the arm*/
  bool helpDisplayed; /*!< flag so help is not repeatedly displayed*/
  bool deadman; /*!< save state of deadman switch */
  bool use_teleop_safety; /*!< launch param to determine which node to publish command velocities to */
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
