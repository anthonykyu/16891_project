#pragma once

#include <vector>
#include <unordered_map>
#include "mamp_planning/edge.hpp"

#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/planning_scene/planning_scene.h>
#include <moveit/kinematic_constraints/utils.h>

// Question: How are we going to store the constraints?

struct Constraint
{
  unsigned int agent_id;
  std::shared_ptr<Edge> joint_pos_edge;
  double time_step;
};

struct Collision
{
  unsigned int agent_id1;
  unsigned int agent_id2;
  std::shared_ptr<Edge> location1;
  std::shared_ptr<Edge> location2;
  double timestep;
};

// helper function file for detect_collisions function
class MAMP_Helper
{
  public:
  // This function is used in the CT node to detect agent-agent collisions
  // The input is an unordered map, with the key being the agent id, and the value is the agent discretized path
  static std::vector<Collision> detectAgentAgentCollisions(std::unordered_map<unsigned int, std::vector<std::shared_ptr<Vertex>>> paths)
  {
    return std::vector<Collision>();
  }

  // check to see if joint space of vertex is out of upper or lower limits
  static bool validJointPos(std::shared_ptr<Vertex> vertex, std::vector<double> &jointUpperLimit, std::vector<double> &jointLowerLimit)
  {
    return true;
  }

  // Use this function in PRM to detect whether an edge is valid; it will discretize the edge and check for collisions with the environment
  static bool detectEdgeCollision(std::shared_ptr<planning_scene::PlanningScene> planning_scene,
                           std::shared_ptr<Edge> edge,
                           std::vector<double> jointVelLimit,
                           double timestep)
  {
    return true;
  }

  // Use this function to detect if a vertex is colliding with the environment
  static bool detectVertexCollision(std::shared_ptr<planning_scene::PlanningScene> planning_scene, std::shared_ptr<Vertex> vertex)
  {
    return true;
  }

  // Use this function to discretize an edge into smaller vertices based on the max velocity limit and timestep
  static std::vector<std::shared_ptr<Vertex>> discretizeEdge(std::shared_ptr<Edge> edge, std::vector<double> jointVelLimit, double timestep)
  {
    return std::vector<std::shared_ptr<Vertex>>();
  }
};
