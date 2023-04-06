#pragma once

#pragma once

#include "ros/ros.h"
#include "mamp_planning/agent.hpp"
#include "mamp_planning/ct_node.hpp"

#define PLANNER_RATE 10

class CBSMP
{
    public:
    CBSMP();


    private:
    void timerCallback(const ros::TimerEvent&);

    ros::NodeHandle n_;
    ros::Timer timer_;

    bool initialized_;
    unsigned int sampling_iter_;
    OpenList<std::shared_ptr<CTNode>, CompareCTNode> open_list_;

};

