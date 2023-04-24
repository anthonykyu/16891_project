#include "mamp_planning/edge.hpp"

Edge::Edge(std::shared_ptr<Vertex> v1, std::shared_ptr<Vertex> v2)
{
  ordered_vertices_.push_back(v1);
  ordered_vertices_.push_back(v2);
  vertices_.insert({v1->getId(), v2});
  vertices_.insert({v2->getId(), v1});
  cost_ = 0;
  for (int i = 0; i < v1->getJointPos().size(); ++i)
  {
    double d = v1->getJointPos()[i] - v2->getJointPos()[i];
    cost_ += d*d;
  }
  cost_ = sqrt(cost_);
  valid_ = true;
  traversal_time_ = 0;
  divisions_ = 0;
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

void Edge::setDivisions(double divisions)
{
  divisions_=divisions;
}

double Edge::getDivisions()
{
  return divisions_;
}



std::shared_ptr<std::vector<std::vector<double>>> Edge::getVertexPositions()
{
  auto vertex_list = std::make_shared<std::vector<std::vector<double>>>(); //(new std::vector<Vertex>>);

  for (std::shared_ptr<Vertex> x : ordered_vertices_)
  {
    // printf("size of joint pos %d", x->getJointPos().size());
    // printf("size of joint pos %ld", x->getId());
    vertex_list->push_back(x->getJointPos());
  }

  return vertex_list;
}


std::shared_ptr<std::vector<std::vector<double>>> Edge::getVertexPositionsInGivenOrder(std::shared_ptr<Vertex> v1, std::shared_ptr<Vertex> v2)
{
  // std::shared_ptr<std::vector<std::vector<double>>> vertex_list; //(new std::vector<Vertex>>);
  auto vertex_list = std::make_shared<std::vector<std::vector<double>>>(); //(new std::vector<Vertex>>);
  for (std::shared_ptr<Vertex> x : {v1, v2})
  {
    vertex_list->push_back(x->getJointPos());
  }

  return vertex_list;
}



std::pair<double, std::vector<double>> Edge::getMagnitudeAndUnitVector(std::shared_ptr<Vertex> start_vertex)
{
  printf("In get magnitude and unit vector function\n");
  std::shared_ptr<std::vector<std::vector<double>>> positions = getVertexPositionsInGivenOrder(start_vertex, getOpposingVertex(start_vertex));

  printf("Got past the first helper\n");
  

  int num_joints = positions->at(0).size();
  // std::vector<double> diff(num_joints);
  double magnitude = 0;
  
  printf("Starting magnitude calculation\n");



  for (int j=0; j<num_joints; ++j)
  {
    // diff[j] = positions->at(1)[j] - positions->at(0)[j];
    // magnitude += diff[j] * diff[j];
    magnitude += (positions->at(1)[j] - positions->at(0)[j]) * (positions->at(1)[j] - positions->at(0)[j]);
  }
  magnitude = std::sqrt(magnitude);

  printf("Finished with magnitude calculation\n");


  // Calculate the unit vector
  std::vector<double> unit_vector(num_joints);
  for (int j=0; j<num_joints; ++j)
  {
      unit_vector[j] = (positions->at(1)[j] - positions->at(0)[j]) / magnitude;
  }

  printf("About to return\n");


  return std::pair<double, std::vector<double>>(magnitude, unit_vector);
}
