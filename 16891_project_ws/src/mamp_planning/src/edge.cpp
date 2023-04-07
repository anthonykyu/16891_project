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

void Edge::setTraversalTime(double t)
{
  traversal_time_ = t;
}

double Edge::getTraversalTime()
{
  return traversal_time_;
}

std::shared_ptr<std::vector<std::vector<double>>> Edge::getVertices()
{
  std::shared_ptr<std::vector<std::vector<double>>> vertex_list; //(new std::vector<Vertex>>);
  for (auto x : vertices_)
  {
    vertex_list->push_back(x.second->getJointPos());
  }

  return vertex_list;
}