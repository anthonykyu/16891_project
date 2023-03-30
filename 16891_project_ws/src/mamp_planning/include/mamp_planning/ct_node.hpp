#pragma once

#include "mamp_planning/agent.hpp"
#include "mamp_planning/mamp_helper.hpp"


// Question: How are we going to store the constraints?
struct constraint
{
    int agent_id;
    vector<double> joint_pos;
    int time_step;

};

class CTNode
{
public:
    CTNode();
    //TODO: add constraints for each agent




private:

};