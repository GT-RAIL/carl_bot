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

  lookAtPointServer = node.advertiseService("asus_controller/look_at_point", &servoPanTilt::lookAtPoint, this);
  lookAtFrameServer = node.advertiseService("asus_controller/look_at_frame", &servoPanTilt::lookAtFrame, this);

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

bool servoPanTilt::lookAtPoint(carl_dynamixel::LookAtPoint::Request &req, carl_dynamixel::LookAtPoint::Response &res)
{
  geometry_msgs::PointStamped transformedPoint;
  tfListener.transformPoint("base_footprint", req.targetPoint, transformedPoint);

  float servoAngle = calculateLookAngle(transformedPoint.point.x, transformedPoint.point.z);
  std_msgs::Float64 cmd;
  cmd.data = servoAngle;
  asusServoControllerPublisher.publish(cmd);

  return true;
}

bool servoPanTilt::lookAtFrame(carl_dynamixel::LookAtFrame::Request &req, carl_dynamixel::LookAtFrame::Response &res)
{
  tf::StampedTransform transform;
  tfListener.lookupTransform("base_footprint", req.frame, ros::Time(0), transform);
  float servoAngle = calculateLookAngle(transform.getOrigin().x(), transform.getOrigin().z());
  std_msgs::Float64 cmd;
  cmd.data = servoAngle;
  asusServoControllerPublisher.publish(cmd);

  return true;
}

float servoPanTilt::calculateLookAngle(float x, float z)
{
  tf::StampedTransform transform;
  tfListener.lookupTransform("base_footprint", "asus_servo_arm_link", ros::Time(0), transform);
  float px = x - transform.getOrigin().x();
  float pz = z - transform.getOrigin().z();
  float angle = atan2(pz, px);

  //transform angle to servo's rotation frame
  return -(angle + .7854);
}

int main(int argc, char **argv)
{
  // initialize ROS and the node
  ros::init(argc, argv, "servo_pan_tilt");

  servoPanTilt s;

  ros::spin();

  return EXIT_SUCCESS;
}
