/*!
 * \carl_interactive_manipulation.h
 * \brief Allows for interactive control of the CARL's JACO arm for object manipulation.
 *
 * carl_interactive_manipulation creates a ROS node that displays interactive markers
 * for the CARL's JACO arm and allows control of the end effector position, grasping, 
 * and pickup actions, as well as segmented and recognized object visualization through
 * an interactive marker client.
 *
 * \author David Kent, WPI - davidkent@wpi.edu
 * \date August 20, 2014
 */

#ifndef CARL_INTERACTIVE_MANIPULATION_H_
#define CARL_INTERACTIVE_MANIPULATION_H_

#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>
#include <carl_moveit/ArmAction.h>
#include <interactive_markers/interactive_marker_server.h>
#include <interactive_markers/menu_handler.h>
#include <rail_manipulation_msgs/GripperAction.h>
#include <rail_manipulation_msgs/LiftAction.h>
#include <rail_manipulation_msgs/RecognizeAction.h>
#include <rail_manipulation_msgs/SegmentedObjectList.h>
#include <rail_pick_and_place_msgs/PickupSegmentedObject.h>
#include <rail_segmentation/RemoveObject.h>
#include <wpi_jaco_msgs/CartesianCommand.h>
#include <wpi_jaco_msgs/EStop.h>
#include <wpi_jaco_msgs/GetCartesianPosition.h>
#include <wpi_jaco_msgs/JacoFK.h>
#include <wpi_jaco_msgs/QuaternionToEuler.h>
#include <sensor_msgs/JointState.h>
#include <sensor_msgs/PointCloud.h>
#include <sensor_msgs/point_cloud_conversion.h>
#include <std_srvs/Empty.h>

//over current thresholds
#define J1_THRESHOLD  7.0
#define J2_THRESHOLD  25.5
#define J3_THRESHOLD 14.0
#define J4_THRESHOLD  5.0
#define J5_THRESHOLD  5.0
#define J6_THRESHOLD 3.5
#define F1_THRESHOLD  1.5
#define F2_THRESHOLD  1.5
#define F3_THRESHOLD 1.5

/*!
 * \class jacoInteractiveManipulation
 * \brief Allows for control of the JACO arm with interactive markers.
 *
 * jaco_joy_teleop creates a ROS node that displays interactive markers for
 * the JACO arm, and allows control of the arm's end effector pose and gripper
 * commands through an interactive marker client such as rviz.
 */
class CarlInteractiveManipulation
{

public:

  /**
   * \brief Constructor
   */
  CarlInteractiveManipulation();

  /**
   * \brief Process feedback for the interactive marker on the JACO's end effector
   * @param feedback interactive marker feedback
   */
  void processHandMarkerFeedback(const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback);

  /**
   * /brief Process feedback for objects that can be recognized.
   * @param feedback interactive marker feedback
   */
  void processRecognizeMarkerFeedback(const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback);

  /**
   * /brief Process feedback for objects that can be picked up.
   * @param feedback interactive marker feedback
   */
  void processPickupMarkerFeedback(const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback);

  /**
  * \brief Process feedback for objects that are selected for removal.
  * @param feedback interactive marker feedback
  */
  void processRemoveMarkerFeedback(const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback);

  /**
   * \brief Callback for the joint state listener
   * @param msg new joint state message
   */
  void updateJoints(const sensor_msgs::JointState::ConstPtr& msg);

  /**
   * \brief callback for segmented objects to be displayed
   * @param objectList list of segmented objects for displaying
   */
  void segmentedObjectsCallback(const rail_manipulation_msgs::SegmentedObjectList::ConstPtr& objectList);

  /**
   * \brief clear all segmented objects from the interactive marker server
   */
  void clearSegmentedObjects();

  /**
   * \brief Update the interactive marker on the JACO's end effector to move based on the the current joint state of the arm
   */
  void updateMarkerPosition();

private:

  /**
  * \brief Remove a manipulation object marker
  * @param index object index
  * @return true if object was successfully removed
  */
  bool removeObjectMarker(int index);

  /**
   * \brief Create the interactive marker on the JACO's end effector, including pose controls and menus
   */
  void makeHandMarker();

  /**
   * \brief Send a 0 velocity command to the robot's arm
   */
  void sendStopCommand();

  /**
  * \brief If interactive markers cause the arm to be in a dangerous collision, briefly reverse arm motion
  */
  void armCollisionRecovery();

  ros::NodeHandle n;

  //messages
  ros::Publisher cartesianCmd;
  ros::Publisher segmentedObjectsPublisher;
  ros::Subscriber jointStateSubscriber;
  ros::Subscriber recognizedObjectsSubscriber;

  //services
  ros::ServiceClient armCartesianPositionClient;
  ros::ServiceClient armEStopClient;
  ros::ServiceClient eraseTrajectoriesClient;
  ros::ServiceClient jacoFkClient;  //!< forward kinematics
  ros::ServiceClient qeClient;  //!< rotation representation conversion client
  ros::ServiceClient pickupSegmentedClient;
  ros::ServiceClient removeObjectClient;

  //actionlib
  actionlib::SimpleActionClient<rail_manipulation_msgs::GripperAction> acGripper;
  actionlib::SimpleActionClient<rail_manipulation_msgs::LiftAction> acLift;
  actionlib::SimpleActionClient<carl_moveit::ArmAction> acArm;
  actionlib::SimpleActionClient<rail_manipulation_msgs::RecognizeAction> acRecognize;

  boost::shared_ptr<interactive_markers::InteractiveMarkerServer> imServer; //!< interactive marker server
  interactive_markers::MenuHandler menuHandler; //!< interactive marker menu handler
  interactive_markers::MenuHandler objectMenuHandler; //!< object interactive markers menu handler
  std::vector<interactive_markers::MenuHandler> recognizedMenuHandlers; //!< list of customized menu handlers for recognized objects
  std::vector<visualization_msgs::InteractiveMarker> segmentedObjects;  //!< list of segmented objects as interactive markers
  rail_manipulation_msgs::SegmentedObjectList segmentedObjectList;  //!< list of segmented objects in the rail_manipulation_msgs form
  std::vector<float> joints;  //!< current joint state
  std::vector<float> markerPose; //!< current pose of the gripper marker
  bool lockPose;  //!< flag to stop the arm from updating on pose changes, this is used to prevent the slight movement when left clicking on the center of the marker
  bool movingArm;
  bool disableArmMarkerCommands;
};

#endif
