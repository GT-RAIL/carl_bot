#include <carl_dynamixel/servo_pan_tilt.h>

using namespace std;

servoPanTilt::servoPanTilt()
{
  // ROS topics
  asusServoControllerPublisher = node.advertise<std_msgs::Float64>("asus_controller/command", 1);
  creativeServoControllerPublisher = node.advertise<std_msgs::Float64>("creative_controller/command", 1);
  tiltCommandSubscriber = node.subscribe("asus_controller/tilt", 1, &servoPanTilt::tiltCallback, this);
  panCommandSubscriber = node.subscribe("creative_controller/pan", 1, &servoPanTilt::panCallback, this);
  backJointStateSubscriber = node.subscribe("dynamixel_back", 1, &servoPanTilt::backJointStateCallback, this);
  frontJointStateSubscriber = node.subscribe("dynamixel_front", 1, &servoPanTilt::frontJointStateCallback, this);

  // initialization
  backServoPos = 0.0;
  frontServoPos = 0.0;
}

void servoPanTilt::backJointStateCallback(const sensor_msgs::JointState::ConstPtr& msg)
{
  backServoPos = msg->position[0];
}

void servoPanTilt::frontJointStateCallback(const sensor_msgs::JointState::ConstPtr& msg)
{
  frontServoPos = msg->position[0];
}

void servoPanTilt::tiltCallback(const std_msgs::Float64::ConstPtr& msg)
{
  //convert command to appropriate value assuming commands are coming in at 30 Hz
  std_msgs::Float64 tiltCommand;
  tiltCommand.data = backServoPos + msg->data/30.0;
  
  //send command to the servo
  asusServoControllerPublisher.publish(tiltCommand);
}

void servoPanTilt::panCallback(const std_msgs::Float64::ConstPtr& msg)
{
  //convert command to appropriate value assuming commands are coming in at 30 Hz
  std_msgs::Float64 panCommand;
  panCommand.data = frontServoPos + msg->data/30.0;
  
  //send command to the servo
  creativeServoControllerPublisher.publish(panCommand);
}

int main(int argc, char **argv)
{
  // initialize ROS and the node
  ros::init(argc, argv, "servo_pan_tilt");

  servoPanTilt s;

  ros::spin();

  return EXIT_SUCCESS;
}
