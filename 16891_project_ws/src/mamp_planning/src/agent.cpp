#include "mamp_planning/agent.hpp"

Agent::Agent(const std::string &robot_description, const std::string &collision_robot_description,
             const std::string &base_frame, const std::string &tip_frame, unsigned int id, std::string &planning_group,
             double &timestep, std::vector<double> start, std::vector<double> goal)
{
  if (!urdf_model_.initString(robot_description))
  {
    ROS_ERROR("Could not initialize tree object");
  }
  if (!compute_joint_limits(base_frame, tip_frame))
  {
    ROS_ERROR("Could not initialize joint limits");
  }
  id_ = id;
  planning_group_ = planning_group;

  robot_model_loader_ = std::make_shared<robot_model_loader::RobotModelLoader>(collision_robot_description);
  kinematic_model_ = std::make_shared<moveit::core::RobotModelPtr>(robot_model_loader_->getModel());
  planning_scene_ = std::make_shared<planning_scene::PlanningScene>(*kinematic_model_);

  timestep_ = timestep;
  path_cost_ = std::numeric_limits<double>::infinity();
  start_ = std::make_shared<Vertex>(start, 0);
  goal_ = std::make_shared<Vertex>(goal, 0);
  prm_ = std::make_shared<PRM>(planning_group_, planning_scene_, timestep_,
                               joint_vel_limit_, upper_joint_limit_, lower_joint_limit_,
                               start_, goal_);
  astar_ = std::make_shared<AStar>(prm_);
}

std::shared_ptr<planning_scene::PlanningScene> const &Agent::getPlanningScene()
{
  return planning_scene_;
}

unsigned int const &Agent::getID()
{
  return id_;
}

std::string const &Agent::getPlanningGroup()
{
  return planning_group_;
}

std::shared_ptr<PRM> &Agent::getPRM()
{
  return prm_;
}

std::vector<double> const &Agent::getJointVelLimit()
{
  return joint_vel_limit_;
}

std::vector<double> const &Agent::getUpperJointLimit()
{
  return upper_joint_limit_;
}

std::vector<double> const &Agent::getLowerJointLimit()
{
  return lower_joint_limit_;
}

std::shared_ptr<Vertex> const &Agent::getStart()
{
  return start_;
}

std::shared_ptr<Vertex> const &Agent::getGoal()
{
  return goal_;
}

bool Agent::compute_joint_limits(const std::string &base_frame, const std::string &tip_frame)
{
  // get joint maxs and mins
  std::shared_ptr<const urdf::Link> link = urdf_model_.getLink(tip_frame);
  std::shared_ptr<const urdf::Joint> joint;
  while (link && link->name != base_frame)
  {
    // std::cout << link->name << std::endl;
    joint = urdf_model_.getJoint(link->parent_joint->name);
    if (!joint)
    {
      ROS_ERROR("Could not find joint: %s", link->parent_joint->name.c_str());
      return false;
    }
    if (joint->type != urdf::Joint::UNKNOWN && joint->type != urdf::Joint::FIXED)
    {
      if (joint->type != urdf::Joint::CONTINUOUS)
      {
        lower_joint_limit_.push_back(joint->limits->lower);
        upper_joint_limit_.push_back(joint->limits->upper);
      }
      else
      {
        lower_joint_limit_.push_back(-std::numeric_limits<double>::infinity());
        upper_joint_limit_.push_back(std::numeric_limits<double>::infinity());
      }

      joint_vel_limit_.push_back(joint->limits->velocity);
    }
    link = urdf_model_.getLink(link->getParent()->name);
  }
  std::reverse(lower_joint_limit_.begin(), lower_joint_limit_.end());
  std::reverse(upper_joint_limit_.begin(), upper_joint_limit_.end());
  std::reverse(joint_vel_limit_.begin(), joint_vel_limit_.end());
  return true;
}