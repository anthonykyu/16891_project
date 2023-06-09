#pragma once

#include <vector>
#include <memory>
#include <limits>
#include <algorithm>
#include <urdf_model/model.h>
#include <urdf/model.h>

#include "mamp_planning/astar.hpp"
#include "mamp_planning/prm.hpp"
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/planning_scene/planning_scene.h>
#include <moveit/kinematic_constraints/utils.h>

class Agent
{
public:
    Agent(const std::string &robot_description, const std::string &collision_robot_description,
        const std::string &base_frame, const std::string &tip_frame, std::string &id,
        double &timestep, std::vector<double> start, std::vector<double> goal);
    Agent(std::shared_ptr<Agent> &a);

    std::shared_ptr<planning_scene::PlanningScene> const &getPlanningScene();
    std::string const &getID();
    std::shared_ptr<PRM> &getPRM();
    std::vector<double> const &getJointVelLimit();
    std::vector<double> const &getUpperJointLimit();
    std::vector<double> const &getLowerJointLimit();
    std::shared_ptr<Vertex> const &getStart();
    std::shared_ptr<Vertex> const &getGoal();
    urdf::Model const &getURDF();
    double const &getTimestep();
    std::shared_ptr<AStar> &getAStar();
    bool computeSingleAgentPath(std::unordered_map<std::shared_ptr<Edge>, Constraint> constraints);
    std::vector<std::shared_ptr<Vertex>> getPRMPath();
    std::vector<std::shared_ptr<Vertex>> getDiscretizedPath();
    // double getPathCost();
    std::shared_ptr<robot_model_loader::RobotModelLoader> const &getRobotModelLoader();
    std::shared_ptr<moveit::core::RobotModelPtr> const &getKinematicModel();
    // collision_detection::AllowedCollisionMatrix const &getACM();


private:
    std::string id_;
    urdf::Model urdf_model_;
    std::vector<double> upper_joint_limit_;
    std::vector<double> lower_joint_limit_;
    std::vector<double> joint_vel_limit_;
    std::shared_ptr<Vertex> start_;
    std::shared_ptr<Vertex> goal_;
    std::shared_ptr<PRM> prm_;
    std::shared_ptr<AStar> astar_;
    double timestep_;
    std::vector<std::shared_ptr<Vertex>> prm_path_;
    std::vector<std::shared_ptr<Vertex>> discretized_path_;
    // double path_cost_;

    std::shared_ptr<robot_model_loader::RobotModelLoader> robot_model_loader_;
    std::shared_ptr<moveit::core::RobotModelPtr> kinematic_model_;
    std::shared_ptr<planning_scene::PlanningScene> planning_scene_;
    // collision_detection::CollisionRequest collision_request_;
    // collision_detection::CollisionResult collision_result_;
    // std::shared_ptr<moveit::core::RobotState> current_state_;
    // collision_detection::AllowedCollisionMatrix acm_; // allowed collision matrix

    bool compute_joint_limits(const std::string& base_frame, const std::string& tip_frame);
};