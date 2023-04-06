#include "mamp_planning/mamp_helper.hpp"



MAMP_Helper::MAMP_Helper(const std::string &full_world_description)
// Take in the parameter describing the whole world
// Load up the planning scene for the entire world
{
    robot_model_loader::RobotModelLoader robot_model_loader(full_world_description);
    const moveit::core::RobotModelPtr& kinematic_model = robot_model_loader.getModel();
    planning_scene_ = planning_scene::PlanningScene(kinematic_model);
}