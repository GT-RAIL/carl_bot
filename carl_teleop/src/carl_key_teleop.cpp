/*!
* \carl_key_teleop.cpp
* \brief Allows for control of CARL with a keyboard.
*
* carl_joy_teleop creates a ROS node that allows the control of CARL with a keyboard.
* This node listens to a /joy topic and sends messages to the /cmd_vel topic.
*
* \author Steven Kordell, WPI - spkordell@wpi.edu
* \date May 23, 2014
*/

#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <signal.h>
#include <termios.h>
#include <stdio.h>
#include "boost/thread/mutex.hpp"
#include "boost/thread/thread.hpp"
#include <carl_teleop/carl_key_teleop.h>

#define KEYCODE_R 0x43
#define KEYCODE_L 0x44
#define KEYCODE_U 0x41
#define KEYCODE_D 0x42
#define KEYCODE_Q 0x71


carl_key_teleop::carl_key_teleop():
  ph_("~"),
  linear_(0),
  angular_(0),
  l_scale_(1.0),
  a_scale_(1.0)
{
  ph_.param("scale_angular", a_scale_, a_scale_);
  ph_.param("scale_linear", l_scale_, l_scale_);

  vel_pub_ = nh_.advertise<geometry_msgs::Twist>("cmd_vel", 1);
}

int kfd = 0;
struct termios cooked, raw;

void quit(int sig)
{
  tcsetattr(kfd, TCSANOW, &cooked);
  ros::shutdown();
  exit(0);
}


int main(int argc, char** argv)
{
  ros::init(argc, argv, "carl_key_teleop");
  carl_key_teleop key_controller;
  ros::NodeHandle n;

  signal(SIGINT,quit);

  boost::thread my_thread(boost::bind(&carl_key_teleop::keyLoop, &key_controller));

  ros::Timer timer = n.createTimer(ros::Duration(0.1), boost::bind(&carl_key_teleop::watchdog, &key_controller));

  ros::spin();

  my_thread.interrupt() ;
  my_thread.join() ;

  return(0);
}


void carl_key_teleop::watchdog()
{
  boost::mutex::scoped_lock lock(publish_mutex_);
  if ((ros::Time::now() > last_publish_ + ros::Duration(0.15)) &&
      (ros::Time::now() > first_publish_ + ros::Duration(0.50)))
    publish(0, 0);
}

void carl_key_teleop::keyLoop()
{
  char c;


  // get the console in raw mode
  tcgetattr(kfd, &cooked);
  memcpy(&raw, &cooked, sizeof(struct termios));
  raw.c_lflag &=~ (ICANON | ECHO);
  // Setting a new line, then end of file
  raw.c_cc[VEOL] = 1;
  raw.c_cc[VEOF] = 2;
  tcsetattr(kfd, TCSANOW, &raw);

  puts("Reading from keyboard");
  puts("---------------------------");
  puts("Use arrow keys to move carl.");


  while (ros::ok())
  {
    // get the next event from the keyboard
    if(read(kfd, &c, 1) < 0)
    {
      perror("read():");
      exit(-1);
    }


    linear_=angular_=0;
    ROS_DEBUG("value: 0x%02X\n", c);

    switch(c)
    {
      case KEYCODE_L:
        ROS_DEBUG("LEFT");
        angular_ = 1.0;
        break;
      case KEYCODE_R:
        ROS_DEBUG("RIGHT");
        angular_ = -1.0;
        break;
      case KEYCODE_U:
        ROS_DEBUG("UP");
        linear_ = 1.0;
        break;
      case KEYCODE_D:
        ROS_DEBUG("DOWN");
        linear_ = -1.0;
        break;
    }
    boost::mutex::scoped_lock lock(publish_mutex_);
    if (ros::Time::now() > last_publish_ + ros::Duration(1.0)) {
      first_publish_ = ros::Time::now();
    }
    last_publish_ = ros::Time::now();
    publish(angular_, linear_);
  }

  return;
}

void carl_key_teleop::publish(double angular, double linear)
{
    geometry_msgs::Twist vel;
    vel.angular.z = a_scale_*angular;
    vel.linear.x = l_scale_*linear;

    vel_pub_.publish(vel);


  return;
}
