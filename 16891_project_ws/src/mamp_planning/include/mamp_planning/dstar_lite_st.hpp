#pragma once

#include "mamp_planning/open_list.hpp"
#include "mamp_planning/prm.hpp"
#include <unordered_map>
#include <unordered_set>
#include <algorithm>
#include <limits>
#include <cmath>

class DStarLiteST
{
public:
    DStarLiteST(double timestep);
    DStarLiteST(std::shared_ptr<DStarLiteST> &d);
    // bool computeWaypointPaths(std::vector<std::shared_ptr<Vertex>> waypoints,
    //                     std::pair<std::unordered_map<std::shared_ptr<Vertex>, std::vector<Constraint>>, std::unordered_map<std::shared_ptr<Edge>, std::vector<Constraint>>> &constraints,
    //                     double max_constraint_time = 0);
    bool computePRMPath(std::shared_ptr<Vertex> start, std::shared_ptr<Vertex> goal,
                        std::pair<std::unordered_map<std::shared_ptr<Vertex>, std::vector<Constraint>>, std::unordered_map<std::shared_ptr<Edge>, std::vector<Constraint>>> &constraints,
                        double max_constraint_time = 0, std::vector<Constraint> new_constraints = std::vector<Constraint>(),
                        double start_time = 0, bool is_last_waypoint = true);
    std::vector<std::shared_ptr<Vertex>> getPRMPath(std::shared_ptr<Vertex> start, std::shared_ptr<Vertex> goal,
                        std::pair<std::unordered_map<std::shared_ptr<Vertex>, std::vector<Constraint>>, std::unordered_map<std::shared_ptr<Edge>, std::vector<Constraint>>> &constraints);

    OpenList<std::tuple<double, double, unsigned int, double>, std::tuple<unsigned int, double>, Vertex, hash_tuple::hash<std::tuple<unsigned int, double>>> &getOpenList();
    std::unordered_map<std::tuple<unsigned int, double>, double, hash_tuple::hash<std::tuple<unsigned int, double>>> &getGMap();
    std::unordered_map<std::tuple<unsigned int, double>, double, hash_tuple::hash<std::tuple<unsigned int, double>>> &getRHSMap();
    std::unordered_map<unsigned int, double> &getH();
    double getMaxRoadmapTraversalTime();
    double getTimestep();
    double getPathTime();
    std::set<double> &getExpandedGoalTimes();
    void initialize(std::shared_ptr<Vertex> start, std::shared_ptr<Vertex> goal);

private:
    // the tuple contains: F, id, time
    OpenList<std::tuple<double, double, unsigned int, double>, std::tuple<unsigned int, double>, Vertex, hash_tuple::hash<std::tuple<unsigned int, double>>> open_list_;
    // the tuple contains: id, time
    std::unordered_map<std::tuple<unsigned int, double>, double, hash_tuple::hash<std::tuple<unsigned int, double>>> g_;
    std::unordered_map<std::tuple<unsigned int, double>, double, hash_tuple::hash<std::tuple<unsigned int, double>>> rhs_;
    // std::unordered_map<std::tuple<unsigned int, double>, std::tuple<double, double>, hash_tuple::hash<std::tuple<unsigned int, double>>> key_map_;
    // key: id, value: h-value
    std::unordered_map<unsigned int, double> h_;
    double max_roadmap_traversal_time_;
    double timestep_;
    std::set<double> expanded_goal_times_;
    double path_time_;

    double getG(std::tuple<unsigned int, double> vertex);
    void setG(std::tuple<unsigned int, double> key, double value);
    double getRHS(std::tuple<unsigned int, double> vertex);
    void setRHS(std::tuple<unsigned int, double> key, double value);
    void updateVertex(std::tuple<unsigned int, double> vertex, std::shared_ptr<Vertex> vertex_ptr, std::shared_ptr<Vertex> goal);
    std::tuple<double, double> calculateKey(std::tuple<unsigned int, double> vertex);
    double computeHeuristics(std::shared_ptr<Vertex> goal);
    bool computeShortestPath(std::shared_ptr<Vertex> start, std::shared_ptr<Vertex> goal,
                        std::pair<std::unordered_map<std::shared_ptr<Vertex>, std::vector<Constraint>>, std::unordered_map<std::shared_ptr<Edge>, std::vector<Constraint>>> &constraints,
                        double max_constraint_time = 0);
    bool shortestPathSearchCondition(std::shared_ptr<Vertex> start, std::shared_ptr<Vertex> goal,
                         std::pair<std::unordered_map<std::shared_ptr<Vertex>, std::vector<Constraint>>, std::unordered_map<std::shared_ptr<Edge>, std::vector<Constraint>>> &constraints,
                         double max_constraint_time);
    bool isValid(std::shared_ptr<Vertex> vertex, std::shared_ptr<Edge> edge);
    bool isConstrained(std::shared_ptr<Vertex> vertex, std::shared_ptr<Edge> edge, double current_time,
                        std::pair<std::unordered_map<std::shared_ptr<Vertex>, std::vector<Constraint>>, std::unordered_map<std::shared_ptr<Edge>, std::vector<Constraint>>> &constraints);
    double round(double in);
    void clearExpandedGoals(std::shared_ptr<Vertex> goal);
};