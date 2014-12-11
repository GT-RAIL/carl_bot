/*!
 * \file camera_external_calibration.cpp
 * \brief Calibration of camera transforms from a fixed AR tag on CARL
 *
 * \author David Kent, WPI - davidkent@wpi.edu
 * \date December 10, 2014
 */
#include <carl_tools/camera_external_calibration.h>

using namespace std;

CameraExternalCalibration::CameraExternalCalibration()
{
  // private node handle
  ros::NodeHandle private_nh("~");

  // get the id of the calibration marker
  private_nh.param("calibration_marker_id", markerID, 200);

  calibrated = false;
  calibrationEnabled = false;
  calibrationWritten = false;
  transformSamples.clear();

  asusCommandPublisher = n.advertise<std_msgs::Float64>("asus_controller/command", 1);

  markerSubscriber = n.subscribe("asus_marker_tracker/ar_pose_marker", 1, &CameraExternalCalibration::markerCallback, this);

  //set camera position for calibration
  this->setCameraPos();
}

void CameraExternalCalibration::setCameraPos()
{
  ROS_INFO("Setting camera position and waiting for autofocus/autoexposure...");
  std_msgs::Float64 asusCmd;
  asusCmd.data = 0.5;
  asusCommandPublisher.publish(asusCmd);
  ros::Duration(5.0).sleep(); //give time for servo to move and camera to focus
  calibrationEnabled = true;
  ROS_INFO("Ready to calibrate.");
}

void CameraExternalCalibration::markerCallback(const ar_track_alvar_msgs::AlvarMarkers::ConstPtr& msg)
{
  if (calibrationEnabled)
  {
    for (unsigned int i = 0; i < msg->markers.size(); i++)
    {
      //check if the marker detected is the calibration marker
      if (msg->markers[i].id == markerID)
      {
        geometry_msgs::PoseStamped sample = msg->markers[i].pose;
        sample.header.frame_id = msg->markers[i].header.frame_id;

        // construct transform from camera to marker
        tf::Transform tfSample;
        tfSample.setOrigin(tf::Vector3(sample.pose.position.x, sample.pose.position.y, sample.pose.position.z));
        tfSample.setRotation(tf::Quaternion(sample.pose.orientation.x, sample.pose.orientation.y, sample.pose.orientation.z, sample.pose.orientation.w).normalize());
        // invert it
        tf::Transform tfSampleInverse = tfSample.inverse();
        ros::Time time = ros::Time::now();
        tf::StampedTransform tfSampleStamped(tfSampleInverse, time, "calibration_link", "calibrated_asus_camera_link");
        br.sendTransform(tfSampleStamped);

        transformSamples.push_back(tfSampleStamped);
        if (transformSamples.size() >= REQUIRED_SAMPLES)
        {
          ROS_INFO("Finished calibration for asus.");
          calibrationEnabled = false;
        }
      }
    }
  }
}

void CameraExternalCalibration::publishTransforms()
{
  // go through each marker
  bool finished = true;

  // publish the average pose from the camera if it's received enough samples
  if (transformSamples.size() >= REQUIRED_SAMPLES)
  {
    if (!calibrated)
    {
      //calculate average pose
      tf::StampedTransform avgTransform;
      avgTransform.frame_id_ = transformSamples[0].frame_id_;
      avgTransform.child_frame_id_ = transformSamples[0].child_frame_id_;
      avgTransform.stamp_ = ros::Time::now();
      float x = 0.0, y = 0.0, z = 0.0;
      tf::Quaternion avgQuat;
      int failedSamples = 0;
      for (unsigned int i = 0; i < transformSamples.size(); i++)
      {
        //check if a transform is empty, this can happen rarely from lookuptransform errors; these should be ignored
        if (transformSamples[i].getOrigin().x() == 0 && transformSamples[i].getOrigin().y() == 0 && transformSamples[i].getOrigin().z() == 0)
        {
          failedSamples ++;
        }
        else
        {
          x += transformSamples[i].getOrigin().x();
          y += transformSamples[i].getOrigin().y();
          z += transformSamples[i].getOrigin().z();
          if (i == 0)
          {
            avgQuat = transformSamples[i].getRotation().normalize();
          }
          else
          {
            avgQuat.slerp(transformSamples[i].getRotation().normalize(), 1.0 / ((float) (i + 1 - failedSamples))).normalize();
          }
        }
      }

      int numSamples = transformSamples.size() - failedSamples;
      avgTransform.setOrigin(tf::Vector3(x/numSamples, y/numSamples, z/numSamples));
      avgTransform.setRotation(avgQuat);

      finalTransform = avgTransform;
      calibrated = true;
    }

    finalTransform.stamp_ = ros::Time::now();
    br.sendTransform(finalTransform);
  }
  else
  {
    finished = false;
  }

  if (finished && !calibrationWritten)
  {
    //calculate the corrected transform
    tf::StampedTransform correctedTransform;
    tfListener.waitForTransform("asus_mount_link", "calibrated_asus_camera_link", ros::Time::now(), ros::Duration(5.0));
    tfListener.lookupTransform("asus_mount_link", "calibrated_asus_camera_link", ros::Time(0), correctedTransform);
    double roll, pitch, yaw;
    tf::Matrix3x3(correctedTransform.getRotation()).getRPY(roll, pitch, yaw);

    //write calibration file
    ROS_INFO("The new calibrated transform between asus_mount_link and camera_link is:\n\n"
        "<origin xyz=\"%f %f %f\" rpy=\"%f %f %f\" />\n",
        correctedTransform.getOrigin().x(), correctedTransform.getOrigin().y(), correctedTransform.getOrigin().z(),
        roll, pitch, yaw);
    ROS_INFO("If this accurate, copy the above line into the carl_asus_camera.urdf.xacro file in the carl_bot package in place of the origin line\n"
        "<xacro:sensor_asus_xtion_pro parent=\"asus_mount_link\">\n"
        "  <origin ... />\n"
        "</xacro:sensor_asus_xtion_pro>");

    calibrationWritten = true;
  }
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "camera_external_calibration");

  CameraExternalCalibration c;

  ros::Rate loop_rate(30);
  while (ros::ok())
  {
    c.publishTransforms();
    ros::spinOnce();
    loop_rate.sleep();
  }
}
