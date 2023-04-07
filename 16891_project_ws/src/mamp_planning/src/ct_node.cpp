#include "mamp_planning/ct_node.hpp"

CTNode::CTNode(unsigned int id, std::vector<std::shared_ptr<Agent>> agents)
{
  cost_ = 0;
  id_ = id;
  agents_ = agents;
}

CTNode::CTNode(unsigned int id, std::shared_ptr<CTNode>  &n)
{
  id_ = id;
  cost_ = 0;
  constraints_ = n->getConstraints();
  paths_ = n->getPaths();
  // agents_ = n->getAgents();
  for (int i = 0; i < n->getAgents().size(); ++i)
  {
    agents_.push_back(std::make_shared<Agent>(n->getAgents()[i]));
  }
}

void CTNode::addConstraint(Constraint c)
{
  constraints_.push_back(c);
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

std::vector<std::shared_ptr<Agent>> CTNode::getAgents()
{
  return agents_;
}

std::tuple<double, unsigned int> CTNode::getComparisonTuple()
{
  return std::make_tuple(cost_, id_);
}

void CTNode::detectCollisions()
{
  collisions_ = MAMP_Helper::detectAgentAgentCollisions(paths_);
}

size_t CTNode::numCollisions()
{
  return collisions_.size();
}

unsigned int CTNode::getId()
{
  return id_;
}