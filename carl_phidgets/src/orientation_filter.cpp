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

  //initialize filter stuff
  prevUpdateTimeTop = ros::Time::now();
  prevUpdateTime = ros::Time::now();
  PPrevTop = 0;
  PPrev[0] = 0;
  PPrev[1] = 0;
}

void OrientationFilter::topImuCallback(const sensor_msgs::Imu::ConstPtr& data)
{
  if (!baseOrientationInitialized)
  {
    ROS_INFO("Waiting for base orientation to be initialized before calculating frame pitch...");
    return;
  }

  /**************************** Read IMU data *****************************/
  //gyro measurement
  float w = data->angular_velocity.y;

  //gyro covariance
  float Q = data->angular_velocity_covariance[4];

  //accelerometer measurement
  float x = data->linear_acceleration.x;
  float y = data->linear_acceleration.y;
  float z = data->linear_acceleration.z;
  float a = atan2(x, z) + jointStates.position[0]; //frame pitch

  //accelerometer covariance
  float R = 100*data->linear_acceleration_covariance[4];
  //NOTE: this value is adjusted to bias the filter towards preferring the gyro measurements

  //Previous orientation
  float thetaPrev = jointStates.position[3];

  //Time step
  float deltaT = (ros::Time::now() - prevUpdateTimeTop).toSec();
  //update time for next time step
  prevUpdateTimeTop = ros::Time::now();

  /******************** Calculate Filtered Orientation ********************/
  float thetaPredicted = thetaPrev + w*deltaT;
  float P = PPrevTop + Q;

  float J = P/(P + R);
  //if significant acceleration other than gravity is detected, don't use the accelerometer measurement
  float pitch;
  if (fabs(G - sqrt(pow(x, 2) + pow(y, 2) + pow(z, 2))) < .05)
    pitch = thetaPredicted + J*(a - thetaPredicted);
  else
    pitch = thetaPredicted;
  //update P for next time step
  PPrevTop = (1 - J)*P;

  /*
  float x = data->linear_acceleration.x; //value is inverted to account for IMU mounting angle
  float z = data->linear_acceleration.z;
  float pitch = atan2(x, z) + jointStates.position[0];
  */

  jointStates.position[2] = -pitch;  //left strut
  jointStates.position[3] = pitch; //right strut
  frameJointStatePublisher.publish(jointStates);
}

void OrientationFilter::baseImuCallback(const sensor_msgs::Imu::ConstPtr& data)
{
  /**************************** Read IMU data *****************************/
  //gyro measurement
  float w[2];
  w[0] = -data->angular_velocity.x; //value is inverted to account for IMU mounting angle
  w[1] = data->angular_velocity.y;

  //gyro covariance
  float Q[2];
  Q[0] = data->angular_velocity_covariance[0];
  Q[1] = data->angular_velocity_covariance[4];

  //accelerometer measurement
  float x = -data->linear_acceleration.x; //value is inverted to account for IMU mounting angle
  float y = data->linear_acceleration.y;
  float z = -data->linear_acceleration.z; //value is inverted to account for IMU mounting angle
  float a[2];
  a[0] = atan2(y, z); //base pitch
  a[1] = atan2(x, z); //base roll

  //accelerometer covariance
  float R[2];
  R[0] = 100*data->linear_acceleration_covariance[0];
  R[1] = 100*data->linear_acceleration_covariance[4];
  //NOTE: these values are adjusted to bias the filter towards preferring the gyro measurements

  //Previous orientation
  float thetaPrev[2];
  thetaPrev[0] = jointStates.position[0];
  thetaPrev[1] = jointStates.position[1];

  //Time step
  float deltaT = (ros::Time::now() - prevUpdateTime).toSec();
  //update time for next time step
  prevUpdateTime = ros::Time::now();

  /******************** Calculate Filtered Orientation ********************/
  float thetaPredicted[2];
  float P[2];
  float J[2];
  thetaPredicted[0] = thetaPrev[0] + w[0]*deltaT;
  thetaPredicted[1] = thetaPrev[1] + w[1]*deltaT;
  P[0] = PPrev[0] + Q[0];
  P[1] = PPrev[1] + Q[1];
  J[0] = P[0]/(P[0] + R[0]);
  J[1] = P[1]/(P[1] + R[1]);

  //if significant acceleration other than gravity is detected, don't use the accelerometer measurement
  if (fabs(G - sqrt(pow(x, 2) + pow(y, 2) + pow(z, 2))) < .05)
  {
    jointStates.position[0] = thetaPredicted[0] + J[0]*(a[0] - thetaPredicted[0]);
    jointStates.position[1] = thetaPredicted[1] + J[1]*(a[1] - thetaPredicted[1]);
  }
  else
  {
    jointStates.position[0] = thetaPredicted[0];
    jointStates.position[1] = thetaPredicted[1];
  }
  //update P for next time step
  PPrev[0] = (1 - J[0])*P[0];
  PPrev[1] = (1 - J[1])*P[1];

  //ROS_INFO("a:%f, w:%f Q:%f, R:%f, P:%f, J:%f, thetaPrev:%f, thetaPredicted:%f, theta:%f", a[1], w[1], Q[1], R[1], P[1], J[1], thetaPrev[1], thetaPredicted[1], jointStates.position[1]);

  /*
  jointStates.position[0] = atan2(y, z);  //base pitch
  jointStates.position[1] = atan2(x, z);  //base roll
  */
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
