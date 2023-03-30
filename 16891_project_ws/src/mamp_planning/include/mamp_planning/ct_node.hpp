#pragma once

#include "mamp_planning/agent.hpp"
#include "mamp_planning/vertex.hpp"
#include "mamp_planning/edge.hpp"
#include "mamp_planning/prm.hpp"


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