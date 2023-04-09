#pragma once

#include <array>
#include <memory>
#include <unordered_map>
#include <cmath>
#include "mamp_planning/vertex.hpp"

class Vertex;
class Edge
{
public:
    Edge(std::shared_ptr<Vertex> v1, std::shared_ptr<Vertex> v2);
    bool const& isValid();
    void changeValidity(bool validity);
    std::shared_ptr<Vertex> getOpposingVertex(std::shared_ptr<Vertex> v);
    double const &getCost();
    void setTraversalTime(double t);
    double getTraversalTime();
    std::shared_ptr<std::vector<std::vector<double>>> getVertexPositions();
    std::shared_ptr<std::vector<std::vector<double>>> getVertexPositionsInGivenOrder(std::shared_ptr<Vertex> v1, std::shared_ptr<Vertex> v2);
    std::pair<double, std::vector<double>> getMagnitudeAndUnitVector(std::shared_ptr<Vertex> start_vertex);

private:
    double cost_;
    bool valid_;
    double traversal_time_;
    
    std::vector<std::shared_ptr<Vertex>> ordered_vertices_;
    std::unordered_map<unsigned int, std::shared_ptr<Vertex>> vertices_;
};