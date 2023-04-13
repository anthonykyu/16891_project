#pragma once

#include "mamp_planning/open_list.hpp"
#include "mamp_planning/vertex.hpp"
#include "mamp_planning/edge.hpp"
#include "mamp_planning/prm.hpp"
#include "mamp_planning/agent.hpp"

class DStarLite
{
public:
    std::shared_ptr<Vertex> start_;
    std::shared_ptr<Vertex> goal_;
    std::shared_ptr<Agent> constraints_;
    std::vector<std::shared_ptr<Vertex>> Dstarpath_;
    DStarLite(double timestep);

    bool computePRMPath(std::shared_ptr<Vertex> start, std::shared_ptr<Vertex> goal,
                        std::pair<std::unordered_map<std::shared_ptr<Vertex>, std::vector<Constraint>>, std::unordered_map<std::shared_ptr<Edge>, std::vector<Constraint>>> &constraints,
                        double max_constraint_time = 0);
    std::vector<std::shared_ptr<Vertex>> getPRMPath(std::shared_ptr<Vertex> start, std::shared_ptr<Vertex> goal);

private:
    // TODO: need a function to calculate the key

    double computeHeuristics(std::shared_ptr<Vertex> goal);
    bool isValid(std::shared_ptr<Vertex> vertex, std::shared_ptr<Edge> edge);
    bool isConstrained(std::shared_ptr<Vertex> vertex, std::shared_ptr<Edge> edge, double current_time,
                        std::pair<std::unordered_map<std::shared_ptr<Vertex>, std::vector<Constraint>>, std::unordered_map<std::shared_ptr<Edge>, std::vector<Constraint>>> &constraints);

};