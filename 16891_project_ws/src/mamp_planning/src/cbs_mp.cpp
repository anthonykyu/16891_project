#include "mamp_planning/cbs_mp.hpp"

CBSMP::CBSMP()
{
  // initialize all agents with planning scenes
  
  for (int i = 0; i < agents_.size(); ++i)
  {
    // TODO:
    // get agents to build prms
    // get agents to find AStar path
    // This could be done on separate threads???
  }

  initialized_ = false;
  S_ = 1;
  timer_ = n_.createTimer(ros::Duration(1.0 / PLANNER_RATE), &CBSMP::timerCallback, this);
  alpha_ = 0.05;
  X_ = 0.5;
}

void CBSMP::timerCallback(const ros::TimerEvent &)
{
  ROS_INFO("In Timer!");
  unsigned int node_id = 0;
  unsigned int N = 0;
  std::shared_ptr<CTNode> root = std::make_shared<CTNode>(++node_id, agents_, mamp_helper_);
  root->detectCollisions();
  root->computeCost();
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
    }
    Collision c = node->getNextCollision();
    std::vector<Constraint> constraints = MAMP_Helper::resolveCollision(c);
    std::shared_ptr<CTNode> n1 = std::make_shared<CTNode>(++node_id, node);
    std::shared_ptr<CTNode> n2 = std::make_shared<CTNode>(++node_id, node);
    n1->addConstraint(constraints[0]);
    n2->addConstraint(constraints[1]);
    // TODO: Get n1 and n2 to recompute paths for specific agents affected
    n1->getAgents().find(constraints[0].agent_id)->second->computeSingleAgentPath(
      MAMP_Helper::getConstraintsForAgent(n1->getConstraints(), constraints[0].agent_id));
    n1->getPaths().erase({constraints[0].agent_id});
    n1->getPaths().insert({constraints[0].agent_id, n1->getAgents().find(constraints[0].agent_id)->second->getPRMPath()});
    n1->computeCost();
    n2->getAgents().find(constraints[1].agent_id)->second->computeSingleAgentPath(
      MAMP_Helper::getConstraintsForAgent(n2->getConstraints(), constraints[1].agent_id));
    n2->getPaths().erase({constraints[1].agent_id});
    n2->getPaths().insert({constraints[1].agent_id, n2->getAgents().find(constraints[1].agent_id)->second->getPRMPath()});
    n2->computeCost();
    // insert n1 and n2 into open list of cbs
    open_list_.insert(n1->getComparisonTuple(), n1);
    open_list_.insert(n2->getComparisonTuple(), n2);
    N += 2;
  }
  // No solution!!!
}

bool CBSMP::shouldResample(unsigned int N)
{
  double p = 1.0 - pow(X_, alpha_*N/S_);
  return ((double)rand()/(double)RAND_MAX) < p;
}


