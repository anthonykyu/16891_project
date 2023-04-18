#include "mamp_planning/dstar_lite.hpp"

DStarLite::DStarLite(double timestep)
{
    timestep_ = timestep;
    path_time_ = -1;
    key_modifier_ = 0;

}

// bool DStarLite::computePRMPath(std::shared_ptr<Vertex> start, std::shared_ptr<Vertex> goal,
//                         std::pair<std::unordered_map<std::shared_ptr<Vertex>, std::vector<Constraint>>, std::unordered_map<std::shared_ptr<Edge>, std::vector<Constraint>>> &constraints,
//                         double max_constraint_time = 0)
// {
//     return true;
// }

std::vector<std::shared_ptr<Vertex>> DStarLite::getPRMPath(std::shared_ptr<Vertex> start, std::shared_ptr<Vertex> goal)
{
    return std::vector<std::shared_ptr<Vertex>>();
}

double DStarLite::computeHeuristics(std::shared_ptr<Vertex> goal)
{
    return 0;
}

bool DStarLite::isValid(std::shared_ptr<Vertex> vertex, std::shared_ptr<Edge> edge)
{
    return true;
}

bool DStarLite::isConstrained(std::shared_ptr<Vertex> vertex, std::shared_ptr<Edge> edge, double current_time,
                              std::pair<std::unordered_map<std::shared_ptr<Vertex>, std::vector<Constraint>>, std::unordered_map<std::shared_ptr<Edge>, std::vector<Constraint>>> &constraints)
{
    return true;
}

double DStarLite::setRHS(std::shared_ptr<Vertex> vertex, double current_time,
                  std::pair<std::unordered_map<std::shared_ptr<Vertex>, std::vector<Constraint>>, std::unordered_map<std::shared_ptr<Edge>, std::vector<Constraint>>> &constraints)
{
    return 0;
}

double DStarLite::getRHS(std::shared_ptr<Vertex> vertex, double current_time,
                  std::pair<std::unordered_map<std::shared_ptr<Vertex>, std::vector<Constraint>>, std::unordered_map<std::shared_ptr<Edge>, std::vector<Constraint>>> &constraints)
{
    return 0;
}

double DStarLite::computeKey(std::shared_ptr<Vertex> vertex, double current_time,
                      std::pair<std::unordered_map<std::shared_ptr<Vertex>, std::vector<Constraint>>, std::unordered_map<std::shared_ptr<Edge>, std::vector<Constraint>>> &constraints)
{
    return 0;
}






