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

  
}

void CBSMP::timerCallback(const ros::TimerEvent &)
{
  ROS_INFO("In Timer!");
  unsigned int id = 0;
  unsigned int N = 0;
  std::shared_ptr<CTNode> root = std::make_shared<CTNode>(++id, agents_);
  root->detectCollisions();
  root->computeCost();
  open_list_.insert(root);
  while (open_list_.size() > 0)
  {
    if (shouldResample(N))
    {
      // TODO: resample routine
      ++S_;
    }
    std::shared_ptr<CTNode> node = open_list_.pop();
    ++N;
    if (node->numCollisions() == 0)
    {
      // TODO: publish path and return
    }
    Collision c = node->getNextCollision();
    std::vector<Constraint> constraints = MAMP_Helper::resolveCollision(c);
    std::shared_ptr<CTNode> n1 = std::make_shared<CTNode>(++id, node);
    std::shared_ptr<CTNode> n2 = std::make_shared<CTNode>(++id, node);
    n1->addConstraint(constraints[0]);
    n2->addConstraint(constraints[1]);
    // TODO: Get n1 and n2 to recompute paths for specific agents affected
    open_list_.insert(n1);
    open_list_.insert(n2);
    N += 2;
  }
  // No solution!!!
}

bool CBSMP::shouldResample(unsigned int N)
{
  // TODO:
  return false;
}
