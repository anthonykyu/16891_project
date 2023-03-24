#include "mamp_planning/edge.hpp"

Edge::Edge(std::shared_ptr<Vertex> v1, std::shared_ptr<Vertex> v2)
{
  vertices_.insert({v1->getId(), v2});
  vertices_.insert({v2->getId(), v1});
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
