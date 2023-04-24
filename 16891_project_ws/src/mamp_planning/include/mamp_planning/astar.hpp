#pragma once

#include "mamp_planning/open_list.hpp"
#include "mamp_planning/prm.hpp"
#include <unordered_map>
#include <unordered_set>
#include <algorithm>
#include <cmath>

class AStar
{
public:
    AStar(double timestep);
    // AStar(std::shared_ptr<PRM> &prm);
    bool computeWaypointPaths(std::vector<std::shared_ptr<Vertex>> waypoints,
                        std::pair<std::unordered_map<std::shared_ptr<Vertex>, std::vector<Constraint>>, std::unordered_map<std::shared_ptr<Edge>, std::vector<Constraint>>> &constraints,
                        double max_constraint_time = 0);
    bool computePRMPath(std::shared_ptr<Vertex> start, std::shared_ptr<Vertex> goal,
                        std::pair<std::unordered_map<std::shared_ptr<Vertex>, std::vector<Constraint>>, std::unordered_map<std::shared_ptr<Edge>, std::vector<Constraint>>> &constraints,
                        double max_constraint_time = 0, double start_time = 0,
                        bool is_last_waypoint = true);
    std::vector<std::shared_ptr<Vertex>> getPRMPath(std::shared_ptr<Vertex> start, std::shared_ptr<Vertex> goal);
    // AStar(std::shared_ptr<AStar> &astar);
    // AStar(std::shared_ptr<Vertex> start, std::shared_ptr<Vertex> goal, std::shared_ptr<Agent> agent, std::vector<Constraint> constraints);
// 
    // void GetPath();
    // double GetEucDistance(std::shared_ptr<Vertex> v1, std::shared_ptr<Vertex> v2);
    // void backtrack(vector<std::shared_ptr<Vertex>> PRMgraph);

private:
    // the tuple contains: F, id, time
    OpenList<std::tuple<double, unsigned int, double>, std::tuple<unsigned int, double>, Vertex, hash_tuple::hash<std::tuple<unsigned int, double>>> open_list_;
    // the tuple contains: id, time
    std::unordered_map<std::tuple<unsigned int, double>, std::shared_ptr<Vertex>, hash_tuple::hash<std::tuple<unsigned int, double>>> closed_list_;
    // the tuple contains: id, time
    std::unordered_map<std::tuple<unsigned int, double>, double, hash_tuple::hash<std::tuple<unsigned int, double>>> g_;
    std::unordered_map<unsigned int, double> h_;
    double timestep_;

    std::unordered_map<std::tuple<std::shared_ptr<Vertex>, double>, std::tuple<std::shared_ptr<Vertex>, double>, hash_tuple::hash<std::tuple<std::shared_ptr<Vertex>, double>>> parent_map_;
    // std::shared_ptr<PRM> prm_;
    // std::unordered_map<std::tuple<unsigned int, double>, double, hash_tuple::hash<std::tuple<unsigned int, double>>> v_;
    double path_time_;
    double computeHeuristics(std::shared_ptr<Vertex> goal);
    bool isValid(std::shared_ptr<Vertex> vertex, std::shared_ptr<Edge> edge);
    bool isConstrained(std::shared_ptr<Vertex> vertex, std::shared_ptr<Edge> edge, double current_time,
                        std::pair<std::unordered_map<std::shared_ptr<Vertex>, std::vector<Constraint>>, std::unordered_map<std::shared_ptr<Edge>, std::vector<Constraint>>> &constraints);
    double round(double in);
};