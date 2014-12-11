/*!
 * \file camera_external_calibration.h
 * \brief Calibration of camera transforms from a fixed AR tag on CARL
 *
 * \author David Kent, WPI - davidkent@wpi.edu
 * \date December 10, 2014
 */

#ifndef CAMERA_EXTERNAL_CALIBRATION_H_
#define CAMERA_EXTERNAL_CALIBRATION_H_

#include <ros/ros.h>
#include <ar_track_alvar_msgs/AlvarMarkers.h>
#include <geometry_msgs/Pose.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include <std_msgs/Float64.h>

#define REQUIRED_SAMPLES 100

class CameraExternalCalibration
{
public:
  /**
  * \brief Constructor
  */
  CameraExternalCalibration();

  void publishTransforms();

private:
  ros::NodeHandle n;

  ros::Subscriber markerSubscriber;
  ros::Publisher asusCommandPublisher;

  tf::TransformListener tfListener;
  tf::TransformBroadcaster br; /*!< main transform broadcaster */

  int markerID;
  std::vector<tf::StampedTransform> transformSamples;
  tf::StampedTransform finalTransform;
  bool calibrated;
  bool calibrationEnabled;
  bool calibrationWritten;

  /**
  * \brief get samples if calibration is active
  *
  * @param msg marker pose data
  */
  void markerCallback(const ar_track_alvar_msgs::AlvarMarkers::ConstPtr& msg);

  /**
  * \brief set the camera position to see the calibration marker
  */
  void setCameraPos();
};

/*!
 * Creates and runs the camera_external_calibration node.
 *
 * \param argc argument count that is passed to ros::init
 * \param argv arguments that are passed to ros::init
 * \return EXIT_SUCCESS if the node runs correctly
 */
int main(int argc, char **argv);

#endif
