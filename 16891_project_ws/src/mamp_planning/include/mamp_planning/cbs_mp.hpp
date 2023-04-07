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
    bool shouldResample(unsigned int N);

    ros::NodeHandle n_;
    ros::Timer timer_;

    bool initialized_;
    unsigned int S_;
    std::vector<std::shared_ptr<Agent>> agents_;
    OpenList<std::tuple<double, unsigned int>, CTNode, hash_tuple::hash<std::tuple<double, unsigned int>>> open_list_;

};

