/*!
 * \orientation_filter.cpp
 * \brief Calculates the roll and pitch of the base and the pitch of the sensor tower from CARL's IMUs.
 *
 * orientation_filter creates a ROS node that measures orientations for the base and sensor tower by combining and
 * filtering accelerometer and gyro data.
 *
 * \author David Kent - davidkent@wpi.edu
 * \date January 13, 2015
 */

#include <carl_phidgets/orientation_filter.h>

OrientationFilter::OrientationFilter()
{
  //initialize joint states
  jointStates.name.push_back("base_footprint_base_link_pitch_joint");
  jointStates.name.push_back("base_footprint_base_link_roll_joint");
  jointStates.name.push_back("left_rear_strut_link_left_rear_strut_top_link_joint");
  jointStates.name.push_back("right_rear_strut_link_right_rear_strut_top_link_joint");
  jointStates.position.resize(4);
  jointStates.velocity.resize(4);
  jointStates.effort.resize(4);

  baseOrientationInitialized = false;

  // create the ROS topics
  frameJointStatePublisher = n.advertise<sensor_msgs::JointState>("frame_joint_states", 1);
  baseImuSubscriber = n.subscribe<sensor_msgs::Imu>("imu_base/data_raw", 1, &OrientationFilter::baseImuCallback, this);
  topImuSubscriber = n.subscribe<sensor_msgs::Imu>("imu_top/data_raw", 1, &OrientationFilter::topImuCallback, this);
}

void OrientationFilter::baseImuCallback(const sensor_msgs::Imu::ConstPtr& data)
{
  if (!baseOrientationInitialized)
  {
    ROS_INFO("Waiting for base orientation to be initialized before calculating frame pitch...");
    return;
  }

  float x = -data->linear_acceleration.x; //value is inverted to account for IMU mounting angle
  float z = data->linear_acceleration.z;
  float pitch = atan2(x, z) - jointStates.position[0];

  jointStates.position[2] = -pitch;  //left strut
  jointStates.position[3] = pitch; //right strut
  frameJointStatePublisher.publish(jointStates);
}

void OrientationFilter::topImuCallback(const sensor_msgs::Imu::ConstPtr& data)
{
  float x = data->linear_acceleration.x;
  float y = data->linear_acceleration.y;
  float z = -data->linear_acceleration.z; //value is inverted to account for IMU mounting angle

  jointStates.position[0] = atan2(x, z);  //base pitch
  jointStates.position[1] = atan2(y, z);  //base roll
  frameJointStatePublisher.publish(jointStates);

  if (!baseOrientationInitialized)
  {
    ROS_INFO("Base orientation initialized!");
    baseOrientationInitialized = true;
  }
}

int main(int argc, char **argv)
{
  // initialize ROS and the node
  ros::init(argc, argv, "orientation_filter");

  // initialize the joystick controller
  OrientationFilter of;

  ros::spin();

  return EXIT_SUCCESS;
}
