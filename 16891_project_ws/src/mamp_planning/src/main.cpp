#include "ros/ros.h"
#include "mamp_planning/cbs_mp.hpp"
#include "mamp_planning/pbs_dstar_mp.hpp"

int main(int argc, char **argv)
{
  ros::init(argc, argv, "steer_bot_planning_node");
  ROS_INFO("Heyyy");
  CBSMP planner_;
  ros::spin();
  return 0;
}