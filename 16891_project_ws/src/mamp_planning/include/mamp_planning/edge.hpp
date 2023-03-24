#pragma once

#include <array>
#include <memory>
#include <unordered_map>
#include "mamp_planning/vertex.hpp"
#include "mamp_planning/agent.hpp"

class Vertex;
class Edge
{
public:
    Edge(std::shared_ptr<Vertex> v1, std::shared_ptr<Vertex> v2);
    bool const& isValid();
    void changeValidity(bool validity);
    std::shared_ptr<Vertex> getOpposingVertex(std::shared_ptr<Vertex> v);

private:
    double cost_;
    bool valid_;
    std::unordered_map<unsigned int, std::shared_ptr<Vertex>> vertices_;
};