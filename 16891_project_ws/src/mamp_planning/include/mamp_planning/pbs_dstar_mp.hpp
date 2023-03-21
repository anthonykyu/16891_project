#pragma once

#pragma once

#include "ros/ros.h"
#include "mamp_planning/prm.hpp"
#include "mamp_planning/dstar_lite.hpp"

#define PLANNER_RATE 10

class PBSDStarMP
{
    public:
    PBSDStarMP();

    private:
    void timerCallback(const ros::TimerEvent&);

    ros::NodeHandle n_;
    ros::Timer timer_;
};
