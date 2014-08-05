#include <carl_dynamixel/servo_pan_tilt.h>

using namespace std;

servoPanTilt::servoPanTilt()
{
  // ROS topics
  asusServoControllerPublisher = node.advertise<std_msgs::Float64>("asus_controller/command", 1);
  tiltCommandSubscriber = node.subscribe("asus_controller/tilt", 1, &servoPanTilt::tiltCallback, this);
  jointStateSubscriber = node.subscribe("dynamixel_back", 1, &servoPanTilt::jointStateCallback, this);

  // initialization
  servoPos = 0.0;
}

void servoPanTilt::jointStateCallback(const sensor_msgs::JointState::ConstPtr& msg)
{
  servoPos = msg->position[0];
}

void servoPanTilt::tiltCallback(const std_msgs::Float64::ConstPtr& msg)
{
  //convert command to appropriate value assuming commands are coming in at 30 Hz
  std_msgs::Float64 tiltCommand;
  tiltCommand.data = servoPos + msg->data/30.0;
  
  //send command to the servo
  asusServoControllerPublisher.publish(tiltCommand);
}

int main(int argc, char **argv)
{
  // initialize ROS and the node
  ros::init(argc, argv, "servo_pan_tilt");

  servoPanTilt s;

  ros::spin();

  return EXIT_SUCCESS;
}
