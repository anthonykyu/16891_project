#pragma once

#include "mamp_planning/open_list.hpp"
#include "mamp_planning/prm.hpp"

class AStar
{
public:
    shared_ptr<Vertex> start_;
    shared_ptr<Vertex> goal_;
    // shared_ptr<Agent> agent_;
    std::vector<constraint> constraints_;
    vector<shared_ptr<Vertex>> Astarpath_;
    AStar();
    // AStar(shared_ptr<Vertex> start, shared_ptr<Vertex> goal, shared_ptr<Agent> agent, std::vector<constraint> constraints);
// 
    // void GetPath();
    // double GetEucDistance(shared_ptr<Vertex> v1, shared_ptr<Vertex> v2);
    // void backtrack(vector<shared_ptr<Vertex>> PRMgraph);

private:

};