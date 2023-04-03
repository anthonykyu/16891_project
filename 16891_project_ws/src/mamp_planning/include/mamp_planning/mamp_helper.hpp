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
  static std::vector<Collision> detectAgentAgentCollisions(std::unordered_map<unsigned int, std::vector<std::shared_ptr<Vertex>>> &paths)
  {
    // Use the global planning scene to step each agent (planning group) 
    // through their respective path (given input). At each timestep,
    // check for collisions and append collisions to the output vector.
    return std::vector<Collision>();
  }

  // check to see if joint space of vertex is out of upper or lower limits
  static bool validJointPos(std::shared_ptr<Vertex> vertex, std::vector<double> &jnt_upper_lim, std::vector<double> &jnt_lower_lim)
  {
    // Agents will have an upper and lower joint limit (for manipulators mainly). 
    // Check if the current joint position at the vertex is valid given the upper
    // and lower limits given as inputs.

    return true;
  }

  // Use this function in PRM to detect whether an edge is valid; it will discretize the edge and check for collisions with the environment
  static bool detectEdgeCollision(std::shared_ptr<planning_scene::PlanningScene> planning_scene,
                           std::shared_ptr<Edge> edge,
                           std::vector<double> &jnt_vel_lim,
                           double timestep)
  {
    // This function will be used in the PRM generation.
    // Essentially this calls the discretizeEdge function to break down the given edge input
    // and then calls detectVertexCollision to check for collisions at the vertex with the environment
    return true;
  }

  // Use this function to detect if a vertex is colliding with the environment
  static bool detectVertexCollision(std::shared_ptr<planning_scene::PlanningScene> planning_scene, std::shared_ptr<Vertex> vertex)
  {
    // Uses the given planning scene and vertex to determine if the agent is colliding
    // with the environment of the planning scene. Returns true if colliding.
    return true;
  }

  // Use this function to discretize an edge into smaller vertices based on the max velocity limit and timestep
  static std::vector<std::shared_ptr<Vertex>> discretizeEdge(std::shared_ptr<Edge> edge, std::vector<double> &jnt_vel_lim, double timestep)
  {
    // This function breaks down a given edge into a list of vertices to check for collisions
    // The discretization is based off of the max joint velocity and timestep given as inputs
    return std::vector<std::shared_ptr<Vertex>>();
  }

  // Use this function to discretize an edge
  static std::vector<std::shared_ptr<Vertex>> discretizeEdgeDirected(std::shared_ptr<Vertex> start_vertex, std::shared_ptr<Edge> edge, std::vector<double> &jnt_vel_lim, double timestep)
  {
    // This function breaks down a given edge into a list of vertices where order matters (starting from start_vertex).
    // This functions the same as discretizeEdge, but will be used when finding a path
    // for an agent. This will be used in A* or D* Lite, and the output are the vertices
    // inserted into the Agent's path.
    return std::vector<std::shared_ptr<Vertex>>();
  }
};
