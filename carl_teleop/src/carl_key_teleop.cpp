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
 * carl_key_teleop creates a ROS node that allows the control of CARL with a keyboard. 
 * This node takes input from the keyboard via the terminal and sends messages to the 
 * /cmd_vel topic for the base and cartesian_cmd for the arm.
 *
 * \author David Kent, WPI - davidkent@wpi.edu
 * \author Steven Kordell, WPI - spkordell@wpi.edu
 * \date May 23, 2014
 */

#include <carl_teleop/carl_key_teleop.h>

// used for capturing keyboard input
int kfd = 0;
struct termios cooked, raw;

carl_key_teleop::carl_key_teleop()
{
  // a private handle for this ROS node (allows retrieval of relative parameters)
  ros::NodeHandle private_nh("~");

  // create the ROS topics
  angular_cmd = nh_.advertise<wpi_jaco_msgs::AngularCommand>("jaco_arm/angular_cmd", 10);
  cartesian_cmd = nh_.advertise<wpi_jaco_msgs::CartesianCommand>("jaco_arm/cartesian_cmd", 10);
  vel_pub_ = nh_.advertise<geometry_msgs::Twist>("cmd_vel", 1);

  // read in throttle values
  private_nh.param<double>("linear_throttle_factor_base", linear_throttle_factor_base, 1.0);
  private_nh.param<double>("angular_throttle_factor_base", angular_throttle_factor_base, 1.0);
  private_nh.param<double>("linear_throttle_factor_arm", linear_throttle_factor_arm, 1.0);
  private_nh.param<double>("angular_throttle_factor_arm", angular_throttle_factor_arm, 1.0);
  private_nh.param<double>("finger_throttle_factor", finger_throttle_factor, 1.0);

  mode = BASE_CONTROL;

  ROS_INFO("CARL Keyboard Teleop Started");
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
  puts("   Press the H key for help ");

  while (ros::ok())
  {
    // get the next event from the keyboard
    char c;
    if (read(kfd, &c, 1) < 0)
    {
      ROS_ERROR("Could not read input from keyboard.");
      exit(-1);
    }

    //Display help message
    if (c == KEYCODE_H)
    {
      displayHelp();
    }

    switch (mode)
    {
      case BASE_CONTROL:
      {
        // determine the speed
        double linear = 0;
        double angular = 0;
        switch (c)
        {
          case KEYCODE_LEFT:
            angular = MAX_ANG_VEL_BASE * angular_throttle_factor_base;
            break;
          case KEYCODE_RIGHT:
            angular = -MAX_ANG_VEL_BASE * angular_throttle_factor_base;
            break;
          case KEYCODE_UP:
            linear = MAX_TRANS_VEL_BASE * linear_throttle_factor_base;
            break;
          case KEYCODE_DOWN:
            linear = -MAX_TRANS_VEL_BASE * linear_throttle_factor_base;
            break;
          case KEYCODE_1:
            mode = ARM_CONTROL;
            ROS_INFO("Activated arm control mode");
            break;
          case KEYCODE_2:
            mode = FINGER_CONTROL;
            ROS_INFO("Activated finger control mode");
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
        break;
      case ARM_CONTROL:
      {
        //initialize twist command
        wpi_jaco_msgs::CartesianCommand cmd;
        cmd.position = false;
        cmd.armCommand = true;
        cmd.fingerCommand = false;
        cmd.repeat = true;
        cmd.arm.linear.x = 0.0;
        cmd.arm.linear.y = 0.0;
        cmd.arm.linear.z = 0.0;
        cmd.arm.angular.x = 0.0;
        cmd.arm.angular.y = 0.0;
        cmd.arm.angular.z = 0.0;

        // w/s control forward/backward translation
        // a/d control left/right translation
        // r/f control up/down translation
        // q/e controls roll
        // up/down controls pitch
        // left/right controls yaw
        switch (c)
        {
          case KEYCODE_W:
            cmd.arm.linear.x = MAX_TRANS_VEL_ARM * linear_throttle_factor_arm;
            break;
          case KEYCODE_S:
            cmd.arm.linear.x = -MAX_TRANS_VEL_ARM * linear_throttle_factor_arm;
            break;
          case KEYCODE_A:
            cmd.arm.linear.y = MAX_TRANS_VEL_ARM * linear_throttle_factor_arm;
            break;
          case KEYCODE_D:
            cmd.arm.linear.y = -MAX_TRANS_VEL_ARM * linear_throttle_factor_arm;
            break;
          case KEYCODE_R:
            cmd.arm.linear.z = MAX_TRANS_VEL_ARM * linear_throttle_factor_arm;
            break;
          case KEYCODE_F:
            cmd.arm.linear.z = -MAX_TRANS_VEL_ARM * linear_throttle_factor_arm;
            break;
          case KEYCODE_Q:
            cmd.arm.angular.z = -MAX_ANG_VEL_ARM * angular_throttle_factor_arm;
            break;
          case KEYCODE_E:
            cmd.arm.angular.z = MAX_ANG_VEL_ARM * angular_throttle_factor_arm;
            break;
          case KEYCODE_UP:
            cmd.arm.angular.x = -MAX_ANG_VEL_ARM * angular_throttle_factor_arm;
            break;
          case KEYCODE_DOWN:
            cmd.arm.angular.x = MAX_ANG_VEL_ARM * angular_throttle_factor_arm;
            break;
          case KEYCODE_LEFT:
            cmd.arm.angular.y = MAX_ANG_VEL_ARM * angular_throttle_factor_arm;
            break;
          case KEYCODE_RIGHT:
            cmd.arm.angular.y = -MAX_ANG_VEL_ARM * angular_throttle_factor_arm;
            break;
          case KEYCODE_2:
            mode = FINGER_CONTROL;
            ROS_INFO("Activated finger control mode");
            break;
          case KEYCODE_3:
            mode = BASE_CONTROL;
            ROS_INFO("Activate base control mode");
            break;
        }

        //publish twist to arm controller
        boost::mutex::scoped_lock lock(publish_mutex_);
        if (ros::Time::now() > last_publish_ + ros::Duration(1.0))
        {
          first_publish_ = ros::Time::now();
        }
        last_publish_ = ros::Time::now();
        cartesian_cmd.publish(cmd);
      }
        break;
      case FINGER_CONTROL:
      {
        //initialize finger command
        wpi_jaco_msgs::AngularCommand cmd;
        cmd.position = false;
        cmd.armCommand = false;
        cmd.fingerCommand = true;
        cmd.repeat = true;
        cmd.fingers.resize(3);
        cmd.fingers[0] = 0.0;
        cmd.fingers[1] = 0.0;
        cmd.fingers[2] = 0.0;

        // q/a controls finger 1
        // w/s controls finger 2
        // e/d controls finger 3
        // r/f controls entire hand
        switch (c)
        {
          case KEYCODE_Q:
            cmd.fingers[0] = -MAX_FINGER_VEL * finger_throttle_factor;
            break;
          case KEYCODE_A:
            cmd.fingers[0] = MAX_FINGER_VEL * finger_throttle_factor;
            break;
          case KEYCODE_W:
            cmd.fingers[1] = -MAX_FINGER_VEL * finger_throttle_factor;
            break;
          case KEYCODE_S:
            cmd.fingers[1] = MAX_FINGER_VEL * finger_throttle_factor;
            break;
          case KEYCODE_E:
            cmd.fingers[2] = -MAX_FINGER_VEL * finger_throttle_factor;
            break;
          case KEYCODE_D:
            cmd.fingers[2] = MAX_FINGER_VEL * finger_throttle_factor;
            break;
          case KEYCODE_R:
            cmd.fingers[0] = -MAX_FINGER_VEL * finger_throttle_factor;
            cmd.fingers[1] = cmd.fingers[0];
            cmd.fingers[2] = cmd.fingers[0];
            break;
          case KEYCODE_F:
            cmd.fingers[0] = MAX_FINGER_VEL * finger_throttle_factor;
            cmd.fingers[1] = cmd.fingers[0];
            cmd.fingers[2] = cmd.fingers[0];
            break;
          case KEYCODE_1:
            mode = ARM_CONTROL;
            ROS_INFO("Activated arm control mode");
            break;
          case KEYCODE_3:
            mode = BASE_CONTROL;
            ROS_INFO("Activate base control mode");
            break;
        }

        //publish twist to finger controller
        boost::mutex::scoped_lock lock(publish_mutex_);
        if (ros::Time::now() > last_publish_ + ros::Duration(1.0))
        {
          first_publish_ = ros::Time::now();
        }
        last_publish_ = ros::Time::now();
        angular_cmd.publish(cmd);
      }
        break;
    }
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

void carl_key_teleop::displayHelp()
{
  switch (mode)
  {
    case ARM_CONTROL:
      puts(" ------------------------------------");
      puts("| CARL Keyboard Teleop Help          |");
      puts("|------------------------------------|*");
      puts("| Current Mode: Arm Control          |*");
      puts("|------------------------------------|*");
      puts("| w/s : forward/backward translation |*");
      puts("| a/d : left/right translation       |*");
      puts("| r/f : up/down translation          |*");
      puts("| q/e : roll                         |*");
      puts("| up/down : pitch                    |*");
      puts("| left/right : yaw                   |*");
      puts("| 1 : switch to Arm Control          |*");
      puts("| 2 : switch to Finger Control       |*");
      puts("| 3 : switch to Base Control         |*");
      puts(" ------------------------------------**");
      puts("  *************************************");
      break;
    case FINGER_CONTROL:
      puts(" ------------------------------------");
      puts("| CARL Keyboard Teleop Help          |");
      puts("|------------------------------------|*");
      puts("| Current Mode: Finger Control       |*");
      puts("|------------------------------------|*");
      puts("| q/a : open/close thumb             |*");
      puts("| w/s : open/close top finger        |*");
      puts("| e/d : open/close bottom finger     |*");
      puts("| r/f : open/close entire hand       |*");
      puts("| 1 : switch to Arm Control          |*");
      puts("| 2 : switch to Finger Control       |*");
      puts("| 3 : switch to Base Control         |*");
      puts(" ------------------------------------**");
      puts("  *************************************");
      break;
    case BASE_CONTROL:
      puts(" ------------------------------------");
      puts("| CARL Keyboard Teleop Help          |");
      puts("|------------------------------------|*");
      puts("| Current Mode: Base Control         |*");
      puts("|------------------------------------|*");
      puts("| up/down : forward/backward         |*");
      puts("| left/right : turn left/right       |*");
      puts("| 1 : switch to Arm Control          |*");
      puts("| 2 : switch to Finger Control       |*");
      puts("| 3 : switch to Base Control         |*");
      puts(" ------------------------------------**");
      puts("  *************************************");
  }
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
