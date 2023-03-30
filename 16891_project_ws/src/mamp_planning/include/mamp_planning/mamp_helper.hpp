#pragma once

#include <vector>
#include "mamp_planning/edge.hpp"

// Question: How are we going to store the constraints?
struct constraint
{
    int agent_id;
    std::vector<double> joint_pos;
    int time_step;

};

// helper function file for detect_collisions function