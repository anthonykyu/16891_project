#include "mamp_planning/agent.hpp"

Agent::Agent(const std::string &robot_description, const std::string &robot_description_name,
             const std::string &base_frame, const std::string &tip_frame, std::string &id,
             double &timestep, std::vector<std::vector<double>> waypoints)
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

  robot_model_loader_ = std::make_shared<robot_model_loader::RobotModelLoader>(robot_description_name);
  kinematic_model_ = std::make_shared<moveit::core::RobotModelPtr>(robot_model_loader_->getModel());
  planning_scene_ = std::make_shared<planning_scene::PlanningScene>(*kinematic_model_);

  timestep_ = timestep;
  // path_cost_ = std::numeric_limits<double>::infinity();
  for (int i = 0; i < waypoints.size(); ++i)
  {
    waypoints_.push_back(std::make_shared<Vertex>(waypoints[i], i));
  }
  
  prm_ = std::make_shared<PRM>(planning_scene_, timestep_,
                               joint_vel_limit_, upper_joint_limit_, lower_joint_limit_,
                               waypoints_);
  astar_ = std::make_shared<AStar>(timestep_);
}

Agent::Agent(std::shared_ptr<Agent> &a)
{
  id_ = a->getID();
  urdf_model_ = a->getURDF();
  upper_joint_limit_ = a->getUpperJointLimit();
  lower_joint_limit_ = a->getLowerJointLimit();
  joint_vel_limit_ = a->getJointVelLimit();
  // start_ = a->getStart();
  // goal_ = a->getGoal();
  waypoints_ = a->getWaypoints();
  prm_ = a->getPRM();
  timestep_ = a->getTimestep();
  prm_path_ = a->getPRMPath();
  discretized_path_ = a->getDiscretizedPath();
  // path_cost_ = a->getPathCost();
  robot_model_loader_ = a->getRobotModelLoader();
  kinematic_model_ = a->getKinematicModel();
  planning_scene_ = a->getPlanningScene();
  // acm_ = a->getACM();
  astar_ = std::make_shared<AStar>(timestep_);
}

bool Agent::checkPath(Constraint new_constraint, std::pair<std::unordered_map<std::shared_ptr<Vertex>, std::vector<Constraint>>, std::unordered_map<std::shared_ptr<Edge>, std::vector<Constraint>>> constraints)
{
  auto temp_prm_path = astar_->getPRMPath(waypoints_[0], waypoints_[waypoints_.size()-1]);
  // ROS_INFO("Got PRM Path!");
  double current_time = 0;
  bool isConstrained = false;

  auto v_constraints = constraints.first;
  auto e_constraints = constraints.second;
  ROS_INFO("Vertex Constraints");
  for (auto x : v_constraints)
  {
    ROS_INFO("Address %p", x.first.get());
  }
  ROS_INFO("Edge Constraints");
  for (auto x : e_constraints)
  {
    ROS_INFO("Address %p", x.first.get());
  }

  if (new_constraint.is_vertex_constraint)
  {
    if (constraints.first.find(new_constraint.joint_pos_vertex) == constraints.first.end())
    {
      ROS_INFO("New Vertex Constraint NOT FOUND!!!");
    }
    else
    {
      ROS_INFO("New Vertex Constraint FOUND!!!");
      for (auto c : constraints.first.find(new_constraint.joint_pos_vertex)->second)
      {
        ROS_INFO("%f, %s, %p", c.time_step, c.agent_id.c_str(), c.joint_pos_vertex.get());
      }
    }
  }
  else
  {
    if (constraints.second.find(new_constraint.joint_pos_edge) == constraints.second.end())
    {
      ROS_INFO("New Edge Constraint NOT FOUND!!!");
    }
    else
    {
      ROS_INFO("New Edge Constraint FOUND!!!");
      for (auto c : constraints.second.find(new_constraint.joint_pos_edge)->second)
      {
        ROS_INFO("%f, %s, %p", c.time_step, c.agent_id.c_str(), c.joint_pos_edge.get());
      }
    }
  }

  for (int i = 0; i < temp_prm_path.size()-1; ++i)
  {
    if (temp_prm_path[i]->getId() != temp_prm_path[i+1]->getId())
    {
      if (new_constraint.time_step <= current_time + prm_path_[i]->getEdges().find(prm_path_[i+1])->second->getTraversalTime() && new_constraint.time_step >= current_time)
      {
        ROS_ERROR("SHOULD TRIGGER, EDGE");
      }

      isConstrained = astar_->isConstrained(temp_prm_path[i+1], prm_path_[i]->getEdges().find(prm_path_[i+1])->second, current_time, constraints, true);
      if (isConstrained)
      {
        ROS_ERROR("IS CONSTRAINED AT TIME %f", current_time);
        return false;
      }
      // std::vector<std::shared_ptr<Vertex>> dv = MAMP_Helper::discretizeEdgeDirected(prm_path_[i], prm_path_[i]->getEdges().find(prm_path_[i+1])->second, joint_vel_limit_, timestep_);
      current_time += prm_path_[i]->getEdges().find(prm_path_[i+1])->second->getTraversalTime();
    }
    else
    {
      if (new_constraint.time_step <= current_time + timestep_ && new_constraint.time_step >= current_time)
      {
        ROS_ERROR("SHOULD TRIGGER, VERTEX");
      }
      isConstrained = astar_->isConstrained(temp_prm_path[i+1], nullptr, current_time, constraints, true);
      if (isConstrained)
      {
        ROS_ERROR("IS CONSTRAINED AT TIME %f", current_time);
        return false;
      }
      current_time += timestep_;
    }
  }
  ROS_INFO("Discretized Path");
  for (int i = std::max((int)(new_constraint.time_step/timestep_) - 20, 0); i < std::min((int)(new_constraint.time_step/timestep_) + 20, (int)discretized_path_.size()); ++i)
  {
    if (discretized_path_[i]->getPRMEdge() != nullptr)
      ROS_INFO("%d, %f, %f" ,i, discretized_path_[i]->getJointPos()[0],discretized_path_[i]->getJointPos()[1]);
    else
      ROS_WARN("%d, %f, %f" ,i, discretized_path_[i]->getJointPos()[0],discretized_path_[i]->getJointPos()[1]);
  }
  return true;
}


bool Agent::computeSingleAgentPath(
  std::pair<std::unordered_map<std::shared_ptr<Vertex>, std::vector<Constraint>>,
  std::unordered_map<std::shared_ptr<Edge>, std::vector<Constraint>>> constraints, 
  double max_constraint_time)
{
  // ROS_INFO("About to clear discretized path");
  discretized_path_.clear();
  // ROS_INFO("Cleared discretized path: %ld", discretized_path_.size());
  // if (astar_->computePRMPath(start_, goal_, constraints, max_constraint_time))
  if (astar_->computeWaypointPaths(waypoints_, constraints, max_constraint_time))
  {
    // ROS_INFO("Computed PRM Path!");

    prm_path_ = astar_->getPRMPath(waypoints_[0], waypoints_[waypoints_.size()-1]);
    // ROS_INFO("Got PRM Path!");
    for (int i = 0; i < prm_path_.size()-1; ++i)
    {
      discretized_path_.push_back(prm_path_[i]);
      // ROS_WARN("ID comparison(%ld, %ld)", prm_path_[i]->getId(), prm_path_[i+1]->getId());
      // for (int j = 0; j < prm_path_[i]->getJointPos().size(); ++j)
      // {
      //   // ROS_INFO("Joint Pos %d: %f versus %f", j, prm_path_[i]->getJointPos()[j], prm_path_[i+1]->getJointPos()[j]);
      // }
      if (prm_path_[i]->getPRMEdge() != nullptr)
        ROS_ERROR("THATS NOT SUPPOSED TO BE HERE");
      if (prm_path_[i]->getId() != prm_path_[i+1]->getId())
      {
        // ROS_INFO("Got into the if statement");
        std::vector<std::shared_ptr<Vertex>> dv = MAMP_Helper::discretizeEdgeDirected(prm_path_[i], prm_path_[i]->getEdges().find(prm_path_[i+1])->second, joint_vel_limit_, timestep_);
        // std::reverse(dv.begin(), dv.end());
        for (auto v : dv)
        {
          discretized_path_.push_back(v);
        }
        // ROS_INFO("dv size: %ld", dv.size());
      }
      // else
      // {
      //   ROS_ERROR("whooopsies");
      // }
    }
    discretized_path_.push_back(prm_path_[prm_path_.size()-1]);
    // for (int i = 0; i < 10; ++i)
    // {
    //   ROS_INFO("PRM Vertex %d", i);
    //   for (auto v : prm_path_[i]->getJointPos())
    //   {
    //     ROS_INFO("%f, ", v);
    //   }
    // }
    // for (int i = 0; i < 10; ++i)
    // {
    //   ROS_INFO("Disc Vertex %d", i);
    //   for (auto v : discretized_path_[i]->getJointPos())
    //   {
    //     ROS_INFO("%f, ", v);
    //   }
    // }
    // for (auto constraint : constraints.second)
    // {
    //   for (int i = 0; i < discretized_path_.size(); ++i)
    //   {
    //     auto v = discretized_path_[i];
    //     if (v->getPRMEdge() && constraint.first == v->getPRMEdge())
    //     {
    //       ROS_ERROR("Constraint not checked!!!, Timestep: %f", i*timestep_);
    //     }
    //   }
    // }
    return true;
  }

  // ROS_ERROR("computeSingleAgentPath: Failed to compute PRM Path");
  return false;
}

std::shared_ptr<planning_scene::PlanningScene> const &Agent::getPlanningScene()
{
  return planning_scene_;
}

std::string const &Agent::getID()
{
  return id_;
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

std::vector<std::shared_ptr<Vertex>> const &Agent::getWaypoints()
{
  return waypoints_;
}

// std::vector<std::shared_ptr<Vertex>> const &Agent::getStart()
// {
//   return start_;
// }

// std::vector<std::shared_ptr<Vertex>> const &Agent::getGoal()
// {
//   return goal_;
// }

urdf::Model const &Agent::getURDF()
{
  return urdf_model_;
}

double const &Agent::getTimestep()
{
  return timestep_;
}

std::shared_ptr<AStar> &Agent::getAStar()
{
  return astar_;
}

std::vector<std::shared_ptr<Vertex>> Agent::getPRMPath()
{
  return prm_path_;
}

std::vector<std::shared_ptr<Vertex>> Agent::getDiscretizedPath()
{
  return discretized_path_;
}

// double Agent::getPathCost()
// {
//   return path_cost_;
// }

std::shared_ptr<robot_model_loader::RobotModelLoader> const &Agent::getRobotModelLoader()
{
  return robot_model_loader_;
}

std::shared_ptr<moveit::core::RobotModelPtr> const &Agent::getKinematicModel()
{
  return kinematic_model_;
}

// collision_detection::AllowedCollisionMatrix const &Agent::getACM()
// {
//   return acm_;
// }

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