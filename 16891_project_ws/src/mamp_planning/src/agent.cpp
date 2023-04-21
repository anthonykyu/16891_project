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

  // acm_ = std::make_shared<collision_detection::AllowedCollisionMatrix>()
  std::shared_ptr<collision_detection::AllowedCollisionMatrix> acm_;
  acm_ = std::make_shared<collision_detection::AllowedCollisionMatrix>(planning_scene_->getAllowedCollisionMatrix());


  // collision_detection::AllowedCollisionMatrix acm = planning_scene_->getAllowedCollisionMatrixNonConst();
  if (std::strcmp(id.substr(0,3).c_str(), "arm") == 0) // If we are dealing with an arm agent, add to the 
  {
    collision_detection::CollisionRequest collision_request;
    collision_detection::CollisionResult collision_result;
    collision_request.contacts = true;
    collision_request.max_contacts = 1000;

    planning_scene_->checkSelfCollision(collision_request, collision_result);
    collision_detection::CollisionResult::ContactMap::const_iterator it2;

    for (it2 = collision_result.contacts.begin(); it2 != collision_result.contacts.end(); ++it2)
    {
      // IF COLLIDING WITH BASE INCORRECTLY COME BACK TO THIS
      // if (std::strcmp(it2->first.first.substr(0,3).c_str(), "arm") != 0 || std::strcmp(it2->first.second.substr(0,3).c_str(), "arm") != 0){
        // continue;
      // }
      acm_->setEntry(it2->first.first, it2->first.second, true);
    }

    // ROS_WARN("Add this many allowed collisions %d", acm_->getSize());
  }



  timestep_ = timestep;
  // path_cost_ = std::numeric_limits<double>::infinity();
  for (int i = 0; i < waypoints.size(); ++i)
  {
    waypoints_.push_back(std::make_shared<Vertex>(waypoints[i], i));
  }
  
  prm_ = std::make_shared<PRM>(planning_scene_, timestep_,
                               joint_vel_limit_, upper_joint_limit_, lower_joint_limit_,
                               waypoints_, acm_);
  astar_ = std::make_shared<AStar>(timestep_);
  dstarlite_ = std::make_shared<DStarLite>(prm_, timestep_);
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
  dstarlite_ = std::make_shared<DStarLite>(prm_, timestep_);
  astar_ = std::make_shared<AStar>(timestep_);
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
  ROS_INFO("Base frame: %s", base_frame.c_str());
  ROS_INFO("Tip frame: %s", tip_frame.c_str());
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