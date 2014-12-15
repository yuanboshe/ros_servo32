/*!
 * Author: Yuanbo She yuanboshe@126.com
 * Group: LAMDA http://lamda.nju.edu.cn
 *
 * Software License Agreement (BSD License)
 *
 * Copyright (c) 2014, Yuanbo She.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * * Redistributions of source code must retain the above copyright notice, this
 *   list of conditions and the following disclaimer.
 *
 * * Redistributions in binary form must reproduce the above copyright notice,
 *   this list of conditions and the following disclaimer in the documentation
 *   and/or other materials provided with the distribution.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#include <ros/ros.h>
#include <signal.h>
#include <geometry_msgs/Twist.h>
#include <sensor_msgs/JointState.h>
#include <ServoController.h>

ros::Publisher cmdVelPub;
ServoController* pServoController;
std::vector<std::string> name(2);
std::vector<int> pin(2);

void jointCallback(const sensor_msgs::JointStateConstPtr& msg)
{
  pServoController->run(msg->name, msg->position, msg->velocity);
}

void shutdown(int sig)
{
  cmdVelPub.publish(geometry_msgs::Twist());
  ROS_INFO("Servo32Controller ended!");
  ros::shutdown();
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "servo_node");
  ros::NodeHandle node;
  ros::Subscriber jointStateSub = node.subscribe("/joint_states", 1, jointCallback);
  ros::Rate loopRate(10);
  signal(SIGINT, shutdown);
  ROS_INFO("Servo32Controller start...");

  // init ServoController
   name[0] = "head_pan_joint";
   name[1] = "head_tilt_joint";
   pin[0] = 0;
   pin[1] = 1;
   pServoController = new ServoController(name, pin, "/dev/ttyUSB0", 115200);

  ros::spin();
  return 0;
}
