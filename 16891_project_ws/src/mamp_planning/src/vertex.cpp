#include "mamp_planning/vertex.hpp"

Vertex::Vertex(std::vector<double> joint_pos, unsigned int id)
{
  setG(std::numeric_limits<double>::infinity());
  setH(std::numeric_limits<double>::infinity());
  setV(std::numeric_limits<double>::infinity());
  joint_pos_ = joint_pos;
  id_ = id;
  parent_vertex_ = nullptr;
  valid_ = true;
}

bool const &Vertex::isValid()
{
  return valid_;
}

double const &Vertex::getG()
{
  return g_;
}

double const &Vertex::getH()
{
  return h_;
}

double const &Vertex::getV()
{
  return v_;
}

double Vertex::getF()
{
  return g_ + h_;
}

unsigned int const &Vertex::getId()
{
  return id_;
}

unsigned int const &Vertex::getComponentId()
{
  return component_id_;
}

std::vector<double> const &Vertex::getJointPos()
{
  return joint_pos_;
}

std::vector<std::shared_ptr<Edge>> const &Vertex::getEdges()
{
  return edges_;
}

void Vertex::changeValidity(bool validity)
{
  valid_ = validity;
}

void Vertex::setId(unsigned int id)
{
  id_ = id;
}

void Vertex::setG(double g)
{
  g_ = g;
}

void Vertex::setH(double h)
{
  h_ = h;
}

void Vertex::setV(double v)
{
  v_ = v;
}

std::shared_ptr<Vertex> Vertex::getParent()
{
  return parent_vertex_;
}

void Vertex::setParent(std::shared_ptr<Vertex> parent)
{
  parent_vertex_ = parent;
}

void Vertex::addEdge(std::shared_ptr<Edge> edge)
{
  edges_.push_back(edge);
}

void Vertex::setJointPos(std::vector<double> joint_pos)
{
  joint_pos_ = joint_pos;
}

void Vertex::setComponentId(unsigned int component_id)
{
  component_id_ = component_id;
}

std::tuple<double, unsigned int> Vertex::getComparisonTuple()
{
  return std::make_tuple(getF(), getId());
}

int Vertex::checkConsistency()
{
  if (g_ == v_)
  {
    return 0;
  }
  else if (g_ < v_)
  {
    return 1;
  }
  else
  {
    return -1;
  }
}
