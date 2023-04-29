#pragma once

#include "mamp_planning/open_list.hpp"
#include "mamp_planning/prm.hpp"
#include <unordered_map>
#include <unordered_set>
#include <algorithm>
#include <limits>
#include <cmath>

class LPAStar
{
public:
    LPAStar(double timestep);
    LPAStar(std::shared_ptr<LPAStar> &d);
    // bool computeWaypointPaths(std::vector<std::shared_ptr<Vertex>> waypoints,
    //                     std::pair<std::unordered_map<std::shared_ptr<Vertex>, std::vector<Constraint>>, std::unordered_map<std::shared_ptr<Edge>, std::vector<Constraint>>> &constraints,
    //                     double max_constraint_time = 0);
    bool computePRMPath(std::shared_ptr<Vertex> start, std::shared_ptr<Vertex> goal,
                        std::pair<std::unordered_map<std::shared_ptr<Vertex>, std::vector<Constraint>>, std::unordered_map<std::shared_ptr<Edge>, std::vector<Constraint>>> &constraints,
                        double max_constraint_time = 0, std::vector<Constraint> new_constraints = std::vector<Constraint>(),
                        double start_time = 0, bool is_last_waypoint = true);
    std::vector<std::shared_ptr<Vertex>> getPRMPath(std::shared_ptr<Vertex> start, std::shared_ptr<Vertex> goal,
                        std::pair<std::unordered_map<std::shared_ptr<Vertex>, std::vector<Constraint>>, std::unordered_map<std::shared_ptr<Edge>, std::vector<Constraint>>> &constraints);

    OpenList<std::tuple<unsigned int, unsigned int, unsigned int, unsigned int>, std::tuple<unsigned int, unsigned int>, Vertex, hash_tuple::hash<std::tuple<unsigned int, unsigned int>>> &getOpenList();
    std::unordered_map<std::tuple<unsigned int, unsigned int>, unsigned int, hash_tuple::hash<std::tuple<unsigned int, unsigned int>>> &getGMap();
    std::unordered_map<std::tuple<unsigned int, unsigned int>, unsigned int, hash_tuple::hash<std::tuple<unsigned int, unsigned int>>> &getRHSMap();
    std::unordered_map<unsigned int, unsigned int> &getH();
    unsigned int getMaxRoadmapTraversalTime();
    double getTimestep();
    unsigned int getPathTime();
    std::set<unsigned int> &getExpandedGoalTimes();
    void initialize(std::shared_ptr<Vertex> start, std::shared_ptr<Vertex> goal);
    unsigned int getNumExpansions();

private:
    // the tuple contains: F, id, time
    OpenList<std::tuple<unsigned int, unsigned int, unsigned int, unsigned int>, std::tuple<unsigned int, unsigned int>, Vertex, hash_tuple::hash<std::tuple<unsigned int, unsigned int>>> open_list_;
    // the tuple contains: id, time
    std::unordered_map<std::tuple<unsigned int, unsigned int>, unsigned int, hash_tuple::hash<std::tuple<unsigned int, unsigned int>>> g_;
    std::unordered_map<std::tuple<unsigned int, unsigned int>, unsigned int, hash_tuple::hash<std::tuple<unsigned int, unsigned int>>> rhs_;
    // std::unordered_map<std::tuple<std::shared_ptr<Vertex>, unsigned int>, std::tuple<std::shared_ptr<Vertex>, unsigned int>, hash_tuple::hash<std::tuple<std::shared_ptr<Vertex>, unsigned int>>> parent_map_;
    // key: id, value: h-value
    std::unordered_map<unsigned int, unsigned int> h_;
    unsigned int max_roadmap_traversal_time_;
    double timestep_;
    std::set<unsigned int> expanded_goal_times_;
    unsigned int path_time_;
    unsigned int num_expansions_;

    unsigned int getG(std::tuple<unsigned int, unsigned int> vertex);
    void setG(std::tuple<unsigned int, unsigned int> key, unsigned int value);
    unsigned int addToG(std::tuple<unsigned int, unsigned int> vertex, unsigned int in);
    unsigned int getRHS(std::tuple<unsigned int, unsigned int> vertex);
    void setRHS(std::tuple<unsigned int, unsigned int> key, unsigned int value);
    void updateVertex(std::tuple<unsigned int, unsigned int> vertex, std::shared_ptr<Vertex> vertex_ptr, std::shared_ptr<Vertex> goal);
    std::tuple<unsigned int, unsigned int> calculateKey(std::tuple<unsigned int, unsigned int> vertex);
    unsigned int computeHeuristics(std::shared_ptr<Vertex> goal);
    bool computeShortestPath(std::shared_ptr<Vertex> start, std::shared_ptr<Vertex> goal,
                        std::pair<std::unordered_map<std::shared_ptr<Vertex>, std::vector<Constraint>>, std::unordered_map<std::shared_ptr<Edge>, std::vector<Constraint>>> &constraints,
                        double max_constraint_time = 0);
    bool shortestPathSearchCondition(std::shared_ptr<Vertex> start, std::shared_ptr<Vertex> goal,
                         std::pair<std::unordered_map<std::shared_ptr<Vertex>, std::vector<Constraint>>, std::unordered_map<std::shared_ptr<Edge>, std::vector<Constraint>>> &constraints,
                         double max_constraint_time);
    bool isValid(std::shared_ptr<Vertex> vertex, std::shared_ptr<Edge> edge);
    bool isConstrained(std::shared_ptr<Vertex> vertex, std::shared_ptr<Edge> edge, unsigned int current_time,
                        std::pair<std::unordered_map<std::shared_ptr<Vertex>, std::vector<Constraint>>, std::unordered_map<std::shared_ptr<Edge>, std::vector<Constraint>>> &constraints);
    double round(double in);
    unsigned int toDiscrete(double in);
    double toContinuous(unsigned int in);
    // void clearExpandedGoals(std::shared_ptr<Vertex> goal);
};