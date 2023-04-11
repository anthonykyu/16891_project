#include "mamp_planning/cbs_mp.hpp"

CBSMP::CBSMP()
{
  initialized_ = false;
  S_ = 1;
  // timer_ = n_.createTimer(ros::Duration(1.0 / PLANNER_RATE), &CBSMP::timerCallback, this);
  alpha_ = 0.05;
  X_ = 0.5;
}

void CBSMP::initialize(std::vector<std::shared_ptr<Agent>> &agents)
{
  // initialize all agents with planning scenes
  for (auto a : agents)
  {
    agents_.insert({a->getID(), a});
  }

  for (auto a : agents_)
  {
    a.second->getPRM()->buildPRM();

    ROS_INFO("PRM Size: %ld", a.second->getPRM()->PRMgraph_.size());
    a.second->computeSingleAgentPath();
  }

  replanCBS();
  initialized_ = true;
}

void CBSMP::timerCallback(const ros::TimerEvent &)
{
  if (!initialized_)
  {
    ROS_INFO("In Timer! Not Initialized Yet!");
    return;
  }
  ROS_INFO("In Timer!");
  
}

bool CBSMP::shouldResample(unsigned int N)
{
  double p = 1.0 - pow(X_, alpha_ * N / S_);
  return ((double)rand()/(double)RAND_MAX) < p;
}

void CBSMP::printPaths(std::shared_ptr<CTNode> node)
{
  ROS_INFO("Number of paths: %ld", node->getPaths().size());
  for (auto p : node->getPaths())
  {
    ROS_INFO("Path for %s", p.first.c_str());
    for (auto v : p.second)
    {
      for (int i = 0; i < v->getJointPos().size(); ++i)
      {
        std::cout << v->getJointPos()[i] << ", ";
      }
      std::cout << std::endl;
    }
  }
}

bool CBSMP::replanCBS()
{
  unsigned int node_id = 0;
  unsigned int N = 0;
  std::shared_ptr<CTNode> root = std::make_shared<CTNode>(++node_id, agents_, mamp_helper_);
  for (auto a : agents_)
  {
    root->getPaths().insert({a.first, a.second->getDiscretizedPath()});
  }

  ROS_INFO("Number of paths: %ld", root->getPaths().size());
  root->detectCollisions();
  ROS_INFO("Number of paths: %ld", root->getPaths().size());
  root->computeCost();
  ROS_INFO("Number of paths: %ld", root->getPaths().size());
  open_list_.insert(root->getComparisonTuple(), root);
  while (open_list_.size() > 0)
  {
    if (shouldResample(N))
    {
      // TODO: resample routine
      ++S_;
    }
    std::shared_ptr<CTNode> node = open_list_.pop().second;
    ++N;
    if (node->numCollisions() == 0)
    {
      // TODO: publish path and return
      ROS_INFO("No collisions!!!");
      ROS_INFO("Number of Constraints: %ld", node->getConstraints().size());
      printPaths(node);
      return true;
    }
    Collision c = node->getNextCollision();
    std::vector<Constraint> constraints = MAMP_Helper::resolveCollision(c);
    std::shared_ptr<CTNode> n1 = std::make_shared<CTNode>(++node_id, node);
    std::shared_ptr<CTNode> n2 = std::make_shared<CTNode>(++node_id, node);
    n1->addConstraint(constraints[0]);
    n2->addConstraint(constraints[1]);
    // TODO: Get n1 and n2 to recompute paths for specific agents affected
    n1->getAgents().find(constraints[0].agent_id)->second->computeSingleAgentPath(
      MAMP_Helper::getConstraintsForAgent(n1->getConstraints(), constraints[0].agent_id), n1->getMaxConstraintTime());
    n1->getPaths().erase({constraints[0].agent_id});
    n1->getPaths().insert({constraints[0].agent_id, n1->getAgents().find(constraints[0].agent_id)->second->getDiscretizedPath()});
    n1->detectCollisions();
    n1->computeCost();
    n2->getAgents().find(constraints[1].agent_id)->second->computeSingleAgentPath(
      MAMP_Helper::getConstraintsForAgent(n2->getConstraints(), constraints[1].agent_id), n2->getMaxConstraintTime());
    n2->getPaths().erase({constraints[1].agent_id});
    n2->getPaths().insert({constraints[1].agent_id, n2->getAgents().find(constraints[1].agent_id)->second->getDiscretizedPath()});
    n2->detectCollisions();
    n2->computeCost();
    // insert n1 and n2 into open list of cbs
    open_list_.insert(n1->getComparisonTuple(), n1);
    open_list_.insert(n2->getComparisonTuple(), n2);
    N += 2;
  }
  // No solution!!!
  return false;
}

