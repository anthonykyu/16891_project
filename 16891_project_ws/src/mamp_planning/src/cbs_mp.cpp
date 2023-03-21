#include "mamp_planning/cbs_mp.hpp"

CBSMP::CBSMP()
{
  timer_ = n_.createTimer(ros::Duration(1.0 / PLANNER_RATE), &CBSMP::timerCallback, this);
}

void CBSMP::timerCallback(const ros::TimerEvent &)
{
  ROS_INFO("In Timer!");
}
