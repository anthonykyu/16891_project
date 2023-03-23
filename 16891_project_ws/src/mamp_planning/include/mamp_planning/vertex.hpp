#pragma once

#include <vector>
#include <memory>
#include "mamp_planning/agent.hpp"
#include "mamp_planning/edge.hpp"

class Edge;
class Vertex
{
public:
    Vertex(std::vector<double> joint_pos, unsigned int id);
    bool const &isValid();
    double const &getG();
    double const &getH();
    double const &getV();
    int const &getId();
    void changeValidity(bool validity);
    void setId(unsigned int id);
    void setG(double g);
    void setH(double h);
    void setV(double v);
    std::shared_ptr<Vertex> getParent();
    void setParent(std::shared_ptr<Vertex> parent);
    void addEdge(std::shared_ptr<Edge>);
    std::tuple<double, unsigned int> getComparisonTuple();
    // function to compare g and v values, -1 underconsistent, 0 consistent, 1 overconsistent
    int checkConsistency();

private:
    double g_;
    double h_;
    double v_;
    bool valid_;
    unsigned int id_;
    std::vector<double> joint_pos_;
    std::shared_ptr<Vertex> parent_vertex_;
    std::vector<std::shared_ptr<Edge>> edges_;
};

struct CompareVertex
{
    bool operator()(const std::shared_ptr<Vertex> &a, const std::shared_ptr<Vertex> &b) const
    {
        return a->getComparisonTuple() < b->getComparisonTuple();
    }
};

// Credit for array hasher function below is from: https://stackoverflow.com/questions/42701688/using-an-unordered-map-with-arrays-as-keys
// struct ArrayHasher
// {
//     std::size_t operator()(const std::array<int, 3> &a) const
//     {
//         std::size_t h = 0;
//         for (auto e : a)
//         {
//             h ^= std::hash<int>{}(e) + 0x9e3779b9 + (h << 6) + (h >> 2);
//         }
//         return h;
//     }
// };