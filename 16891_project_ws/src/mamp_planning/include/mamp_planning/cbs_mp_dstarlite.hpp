#pragma once

#include "ros/ros.h"
#include <omp.h>
#include <chrono>
#include "mamp_planning/agent.hpp"
#include "mamp_planning/ct_node.hpp"

#define PLANNER_RATE 10

class CBSMP_DSTARLITE
{
    public:
    CBSMP_DSTARLITE();
    void initialize(std::vector<std::shared_ptr<Agent>> &agents, std::string &world_planning_scene, double timestep);
    ros::NodeHandle n_;
    std::shared_ptr<MAMP_Helper> get_mamp_helper() {return mamp_helper_;}
    std::vector<std::shared_ptr<Agent>> getAgents();

    private:
    void timerCallback(const ros::TimerEvent&);
    bool shouldResample(unsigned int N);
    bool replanCBS();
    void printPaths(std::shared_ptr<CTNode> node);
    void printConstraints(std::vector<Constraint> node);
    void printCollision(Collision c);

    ros::Timer timer_;

    bool initialized_;
    unsigned int S_;
    double X_;
    double alpha_;
    std::unordered_map<std::string, std::shared_ptr<Agent>> agents_;
    OpenList<std::tuple<double, size_t, unsigned int>, CTNode, hash_tuple::hash<std::tuple<double, size_t, unsigned int>>> open_list_;
    std::shared_ptr<MAMP_Helper> mamp_helper_;
};

