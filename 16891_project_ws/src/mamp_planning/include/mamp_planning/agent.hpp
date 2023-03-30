#pragma once

#include <vector>
#include <memory>
#include <urdf_model/model.h>
#include <urdf/model.h>

#include "mamp_planning/vertex.hpp"
#include "mamp_planning/prm.hpp"
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/planning_scene/planning_scene.h>
#include <moveit/kinematic_constraints/utils.h>

class Agent
{
public:
    Agent(unsigned int id, std::shared_ptr<planning_scene::PlanningScene>& planning_scene);

    std::shared_ptr<planning_scene::PlanningScene> const &getPlanningScene();
    unsigned int const &getID();
    std::shared_ptr<PRM> &getPRM();
    std::vector<double> getJointVelLimit();

private:
    unsigned int id_;
    urdf::Model urdf_model_;
    std::vector<double> upper_joint_limit_;
    std::vector<double> lower_joint_limit_;
    std::vector<double> joint_vel_limit_;
    std::shared_ptr<PRM> prm_;

    std::vector<Vertex> path_;
    std::shared_ptr<robot_model_loader::RobotModelLoader> robot_model_loader_;
    std::shared_ptr<moveit::core::RobotModelPtr> kinematic_model_;
    std::shared_ptr<planning_scene::PlanningScene> planning_scene_;
    // collision_detection::CollisionRequest collision_request_;
    // collision_detection::CollisionResult collision_result_;
    // std::shared_ptr<moveit::core::RobotState> current_state_;
    collision_detection::AllowedCollisionMatrix acm_; // allowed collision matrix

    bool compute_joint_limits();
};