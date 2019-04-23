#include "ros/ros.h"
#include "../include/mmrs_simulator/task.h"

int main(int argc, char *argv[])
{
  ros::init(argc, argv, "simulator");
  ros::NodeHandle node_handle;
  ros::Rate loop_rate(kDefaultSimulationRateHz);

  Task task;

  while(ros::ok())
  {
    if(task.SimulationStep())
    {
      ROS_INFO("Task completed!");
      break;
    }
    ROS_INFO("Hello World!");
    ros::spinOnce();
    loop_rate.sleep();
  }

  return 0;
}
