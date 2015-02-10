/*!
 * \file current_logger.cpp
 * \brief Output of arm joint currents to a csv file
 *
 * \author David Kent, WPI - davidkent@wpi.edu
 * \date February 10, 2015
 */
#include <carl_tools/current_logger.h>

using namespace std;

CurrentLogger::CurrentLogger()
{
  // private node handle
  ros::NodeHandle private_nh("~");

  // get the id of the calibration marker
  private_nh.param<string>("filename", filename, "arm_currents.txt");

  armJointStateSubscriber = n.subscribe("jaco_arm/joint_states", 1, &CurrentLogger::armJointStateCallback, this);
}

void CurrentLogger::armJointStateCallback(const sensor_msgs::JointState::ConstPtr& msg)
{
  ofstream outputFile;
  outputFile.open(filename.c_str(), ios::out | ios::app);
  outputFile << ros::Time::now().toSec() << ",";
  for (unsigned int i = 0; i < msg->effort.size(); i ++)
  {
    if (i < msg->effort.size() - 1)
      outputFile << msg->effort[i] << ",";
    else
      outputFile << msg->effort[i];
  }
  outputFile << "\n";
  outputFile.close();
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "current_logger");

  CurrentLogger cl;

  ros::spin();
}
