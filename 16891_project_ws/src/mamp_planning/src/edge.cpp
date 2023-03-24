#include "mamp_planning/edge.hpp"

Edge::Edge(std::shared_ptr<Vertex> v1, std::shared_ptr<Vertex> v2)
{
  vertices_.insert({v1->getId(), v2});
  vertices_.insert({v2->getId(), v1});
  cost_ = 0;
  for (int i = 0; i < v1->getJointPos().size(); ++i)
  {
    double d = v1->getJointPos()[i] - v2->getJointPos()[i];
    cost_ += d*d;
  }
  cost_ = sqrt(cost_);
}

bool const &Edge::isValid()
{
  return valid_;
}

void Edge::changeValidity(bool validity)
{
  valid_ = validity;
}

std::shared_ptr<Vertex> Edge::getOpposingVertex(std::shared_ptr<Vertex> v)
{
  return vertices_.find(v->getId())->second;
}

double const &Edge::getCost()
{
  return cost_;
}