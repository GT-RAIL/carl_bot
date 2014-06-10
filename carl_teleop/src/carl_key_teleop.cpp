/*
 * Copyright (c) 2011, Willow Garage, Inc.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * * Redistributions of source code must retain the above copyright
 * notice, this list of conditions and the following disclaimer.
 * * Redistributions in binary form must reproduce the above copyright
 * notice, this list of conditions and the following disclaimer in the
 * documentation and/or other materials provided with the distribution.
 * * Neither the name of the Willow Garage, Inc. nor the names of its
 * contributors may be used to endorse or promote products derived from
 * this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

/*!
 * \carl_key_teleop.cpp
 * \brief Allows for control of CARL with a keyboard.
 *
 * carl_joy_teleop creates a ROS node that allows the control of CARL with a keyboard. This node listens to a /joy topic
 * and sends messages to the /cmd_vel topic.
 *
 * \author Steven Kordell, WPI - spkordell@wpi.edu
 * \date May 23, 2014
 */

#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <signal.h>
#include <termios.h>
#include <stdio.h>
#include <boost/thread/mutex.hpp>
#include <boost/thread/thread.hpp>
#include <carl_teleop/carl_key_teleop.h>

// used for capturing keyboard input
int kfd = 0;
struct termios cooked, raw;

carl_key_teleop::carl_key_teleop()
{
  vel_pub_ = nh_.advertise<geometry_msgs::Twist>("cmd_vel", 1);
}

void carl_key_teleop::watchdog()
{
  boost::mutex::scoped_lock lock(publish_mutex_);
  if ((ros::Time::now() > last_publish_ + ros::Duration(0.15))
      && (ros::Time::now() > first_publish_ + ros::Duration(0.50)))
    publish(0, 0);
}

void carl_key_teleop::loop()
{
  // get the console in raw mode
  tcgetattr(kfd, &cooked);
  memcpy(&raw, &cooked, sizeof(struct termios));
  raw.c_lflag &= ~(ICANON | ECHO);
  // Setting a new line, then end of file
  raw.c_cc[VEOL] = 1;
  raw.c_cc[VEOF] = 2;
  tcsetattr(kfd, TCSANOW, &raw);

  puts("    Reading from Keyboard    ");
  puts("-----------------------------");
  puts(" Use Arrow Keys to Move CARL ");

  while (ros::ok())
  {
    // get the next event from the keyboard
    char c;
    if (read(kfd, &c, 1) < 0)
    {
      ROS_ERROR("Could not read input from keyboard.");
      exit(-1);
    }

    // determine the speed
    double linear = 0;
    double angular = 0;
    switch (c)
    {
      case KEYCODE_L:
        angular = MAX_ANG_VEL;
        break;
      case KEYCODE_R:
        angular = -MAX_ANG_VEL;
        break;
      case KEYCODE_U:
        linear = MAX_TRANS_VEL;
        break;
      case KEYCODE_D:
        linear = -MAX_TRANS_VEL;
        break;
    }

    // attempt to publish the speed
    boost::mutex::scoped_lock lock(publish_mutex_);
    if (ros::Time::now() > last_publish_ + ros::Duration(1.0))
    {
      first_publish_ = ros::Time::now();
    }
    last_publish_ = ros::Time::now();
    publish(angular, linear);
  }
}

void carl_key_teleop::publish(double angular, double linear)
{
  geometry_msgs::Twist vel;
  vel.angular.z = angular;
  vel.linear.x = linear;
  // send the command
  vel_pub_.publish(vel);
}

void shutdown(int sig)
{
  // shut everything down
  tcsetattr(kfd, TCSANOW, &cooked);
  ros::shutdown();
}

int main(int argc, char** argv)
{
  // initialize ROS and the node
  ros::init(argc, argv, "carl_key_teleop");

  // initialize the keyboard controller
  carl_key_teleop key_controller;
  ros::NodeHandle n;

  // setup the SIGINT signal for exiting
  signal(SIGINT, shutdown);

  // setup the watchdog and key loop in a thread
  boost::thread my_thread(boost::bind(&carl_key_teleop::loop, &key_controller));
  ros::Timer timer = n.createTimer(ros::Duration(0.1), boost::bind(&carl_key_teleop::watchdog, &key_controller));
  ros::spin();

  // wait for everything to end
  my_thread.interrupt();
  my_thread.join();

  return EXIT_SUCCESS;
}
