#pragma once

#include <vector>
#include <unordered_map>
#include <cstring>
#include "mamp_planning/edge.hpp"

#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/planning_scene/planning_scene.h>
#include <moveit/kinematic_constraints/utils.h>

// Question: How are we going to store the constraints?

struct Constraint
{
  std::string agent_id;
  std::shared_ptr<Edge> joint_pos_edge;
  double time_step;
};

struct Collision
{
  std::string agent_id1;
  std::string agent_id2;
  std::shared_ptr<Edge> location1;
  std::shared_ptr<Edge> location2;
  double timestep;
};

// helper function file for detect_collisions function
class MAMP_Helper
{
  public:
<<<<<<< HEAD
  // This function is used in the CT node to detect agent-agent collisions
  // The input is an unordered map, with the key being the agent id, and the value is the agent discretized path
  static std::vector<Collision> detectAgentAgentCollisions(std::unordered_map<std::string, std::vector<std::shared_ptr<Vertex>>> &paths)
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

  static std::vector<Constraint> resolveCollision(Collision &collision)
  {
    // This function turns a collision into a pair of constraints to be
    // separated into two nodes of a CT for CBS.
    Constraint c1;
    Constraint c2;
    c1.agent_id = collision.agent_id1;
    c1.joint_pos_edge = collision.location1;
    c1.time_step = collision.timestep;
    c2.agent_id = collision.agent_id2;
    c2.joint_pos_edge = collision.location2;
    c2.time_step = collision.timestep;
    std::vector<Constraint> constraints;
    constraints.push_back(c1);
    constraints.push_back(c2);
    return constraints;
  }

  static double getSumOfCosts(std::unordered_map<std::string, std::vector<std::shared_ptr<Vertex>>> &paths)
  {
    // This function adds up costs to get the sum of costs
    double cost = 0;
    for (auto p : paths)
    {
      std::shared_ptr<Vertex> v = p.second[0];
      int i = 1;
      while (i < p.second.size())
      {
        auto it = v->getEdges().find(p.second[i]);
        if (it != v->getEdges().end())
        {
          cost += it->second->getCost();
          v = it->first;
        }
        ++i;
      }
    }
    return cost;
  }

  static std::unordered_map<std::shared_ptr<Edge>, Constraint> getConstraintsForAgent(std::vector<Constraint> constraints, std::string agent)
  {
    std::unordered_map<std::shared_ptr<Edge>, Constraint> m;
    for (int i = 0; i < constraints.size(); ++i)
    {
      if (strcmp(constraints[i].agent_id.c_str(), agent.c_str()) == 0)
      {
        m.insert({constraints[i].joint_pos_edge, constraints[i]});
      }
    }
    return m;
  }
=======
    // CONSTRUCTOR
    MAMP_Helper(const std::string &full_world_description); 
    
    std::shared_ptr<planning_scene::PlanningScene> const &getPlanningScene() {return planning_scene_;}

    
    // Take in inputs of joint positions
    // Check if there is a collision at those joint positions
    // Get back the collisions in the form of the collision result
    // static bool checkCollision(std::vector<>)



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

    static std::vector<Constraint> resolveCollision(Collision &collision)
    {
      // This function turns a collision into a pair of constraints to be
      // separated into two nodes of a CT for CBS.
      return std::vector<Constraint>();
    }

  private:
    static std::shared_ptr<planning_scene::PlanningScene> planning_scene_;

>>>>>>> e31fb5c... Working on adding collision work, comitting to pull in the latest changes from Anthony
};
