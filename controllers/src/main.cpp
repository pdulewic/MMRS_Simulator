// The MIT License (MIT)

// Copyright (c) 2019 Piotr Dulewicz

//  Permission is hereby granted, free of charge, to any person obtaining a
//  copy of this software and associated documentation files (the "Software"),
//  to deal in the Software without restriction, including without limitation
//  the rights to use, copy, modify, merge, publish, distribute, sublicense,
//  and/or sell copies of the Software, and to permit persons to whom the
//  Software is furnished to do so, subject to the following conditions:
//
//  The above copyright notice and this permission notice shall be included in
//  all copies or substantial portions of the Software.
//
//  THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS
//  OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
//  FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
//  AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
//  LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING
//  FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER
//  DEALINGS IN THE SOFTWARE.

/**
 * @file main.cpp
 * @author Piotr Dulewicz (piotr.dulewicz@pwr.edu.pl)
 * @brief dummy controller used only for testing, allows every pass
 * @date 2019-04-23
 * 
 * @copyright Copyright (c) 2019
 * 
 */

#include "ros/ros.h"
#include "std_msgs/Int16.h"

ros::Publisher permission_publisher;

void CriticalPointCallback(const std_msgs::Int16::ConstPtr &msg)
{
  ROS_INFO("critical point reached by vehicle %d", msg->data);
  std_msgs::Int16 output_msg;
  output_msg.data = msg->data;
  permission_publisher.publish(output_msg);
}

void ReleasePointCallback(const std_msgs::Int16::ConstPtr &msg)
{
  ROS_INFO("release point reached by vehicle %d", msg->data);
}

int main(int argc, char *argv[])
{
  ros::init(argc, argv, "dummy_controller");
  ros::NodeHandle node_handle;
  permission_publisher = node_handle.advertise<std_msgs::Int16>(
      "move_permissions", 1000);
  auto critical_point_subscriber = node_handle.subscribe("critical_points",
                                                         1000, CriticalPointCallback);
  auto release_point_subscriber = node_handle.subscribe("release_points",
                                                        1000, ReleasePointCallback);
  ros::spin();
  return 0;
}
