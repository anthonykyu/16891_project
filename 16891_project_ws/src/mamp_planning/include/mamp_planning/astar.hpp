#pragma once

#include "mamp_planning/open_list.hpp"
#include "mamp_planning/prm.hpp"

class AStar
{
public:
    std::shared_ptr<Vertex> start_;
    std::shared_ptr<Vertex> goal_;
    // std::shared_ptr<Agent> agent_;
    std::vector<Constraint> constraints_;
    std::vector<std::shared_ptr<Vertex>> Astarpath_;
    AStar(std::shared_ptr<PRM> &prm);
    // AStar(std::shared_ptr<Vertex> start, std::shared_ptr<Vertex> goal, std::shared_ptr<Agent> agent, std::vector<Constraint> constraints);
// 
    // void GetPath();
    // double GetEucDistance(std::shared_ptr<Vertex> v1, std::shared_ptr<Vertex> v2);
    // void backtrack(vector<std::shared_ptr<Vertex>> PRMgraph);

private:

};