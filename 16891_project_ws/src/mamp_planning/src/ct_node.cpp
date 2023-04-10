#include "mamp_planning/ct_node.hpp"

CTNode::CTNode(unsigned int id, std::unordered_map<std::string, std::shared_ptr<Agent>> agents, std::shared_ptr<MAMP_Helper> &mamp_helper)
{
  cost_ = 0;
  id_ = id;
  agents_ = agents;
  mamp_helper_ = mamp_helper;
  max_constraint_time_ = 0;
}

CTNode::CTNode(unsigned int id, std::shared_ptr<CTNode>  &n)
{
  id_ = id;
  cost_ = 0;
  max_constraint_time_ = n->getMaxConstraintTime();
  constraints_ = n->getConstraints();
  paths_ = n->getPaths();
  mamp_helper_ = n->getMAMPHelper();
  // agents_ = n->getAgents();
  for (auto agent : n->getAgents())
  {
    agents_.insert({agent.first, std::make_shared<Agent>(agent.second)});
  }
}

double CTNode::getMaxConstraintTime()
{
  return max_constraint_time_;
}

std::shared_ptr<MAMP_Helper> &CTNode::getMAMPHelper()
{
  return mamp_helper_;
}

void CTNode::addConstraint(Constraint c)
{
  constraints_.push_back(c);
  if (c.time_step > max_constraint_time_)
  {
    max_constraint_time_ = c.time_step;
  }
}

Collision &CTNode::getNextCollision()
{
  if (collisions_.size() > 0)
    return collisions_[0];
}

void CTNode::computeCost()
{
  cost_ = MAMP_Helper::getSumOfCosts(paths_);
}

std::vector<Constraint> CTNode::getConstraints()
{
  return constraints_;
}

std::unordered_map<std::string, std::vector<std::shared_ptr<Vertex>>> CTNode::getPaths()
{
  return paths_;
}

std::unordered_map<std::string, std::shared_ptr<Agent>> CTNode::getAgents()
{
  return agents_;
}

std::tuple<double, unsigned int> CTNode::getComparisonTuple()
{
  return std::make_tuple(cost_, id_);
}

void CTNode::detectCollisions()
{
  collisions_ = mamp_helper_->detectAgentAgentCollisions(paths_);
}

size_t CTNode::numCollisions()
{
  return collisions_.size();
}

unsigned int CTNode::getId()
{
  return id_;
}