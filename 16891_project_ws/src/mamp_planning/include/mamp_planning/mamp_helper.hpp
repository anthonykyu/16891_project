#pragma once

#include <vector>
#include "mamp_planning/edge.hpp"

// Question: How are we going to store the constraints?
struct constraint
{
    unsigned int int agent_id;
    std::shared_ptr<Edge> joint_pos_edge;
    double time_step;
};

struct collision
{
  unsigned int agent_id1;
  unsigned int agent_id2;
  std::shared_ptr<Edge> location1;
  std::shared_ptr<Edge> location2;
  double timestep;
};

// helper function file for detect_collisions function