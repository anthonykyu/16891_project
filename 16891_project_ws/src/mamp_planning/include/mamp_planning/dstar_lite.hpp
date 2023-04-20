#pragma once

#include "mamp_planning/open_list.hpp"
#include "mamp_planning/prm.hpp"
#include <unordered_map>
#include <unordered_set>
#include <algorithm>
class DStarLite
{
public:
    std::shared_ptr<Vertex> start_;
    std::shared_ptr<Vertex> goal_;
    // std::shared_ptr<Agent> agent_;
    std::vector<Constraint> constraints_;
    std::vector<std::shared_ptr<Vertex>> Astarpath_;
    
    DStarLite(double timestep);
    // AStar(std::shared_ptr<PRM> &prm);
    double computeHeuristics(std::shared_ptr<Vertex> start);
    // void computePRMPath(std::shared_ptr<Vertex> start, std::shared_ptr<Vertex> goal,
    //                     std::pair<std::unordered_map<std::shared_ptr<Vertex>, std::vector<Constraint>>, std::unordered_map<std::shared_ptr<Edge>, std::vector<Constraint>>> &constraints,
    //                     double max_constraint_time);

    // TODO: need to add the constraints
    // void computePRMPath(std::shared_ptr<Vertex> start, std::shared_ptr<Vertex> goal,
    //                     double max_constraint_time);
    void computePRMPath(std::shared_ptr<Vertex> start, std::shared_ptr<Vertex> goal,
                        std::pair<std::unordered_map<std::shared_ptr<Vertex>, std::vector<Constraint>>, std::unordered_map<std::shared_ptr<Edge>, std::vector<Constraint>>> &constraints,
                        double max_constraint_time = 0);
    std::vector<std::shared_ptr<Vertex>> getPRMPath(std::shared_ptr<Vertex> start, std::shared_ptr<Vertex> goal, std::pair<std::unordered_map<std::shared_ptr<Vertex>, std::vector<Constraint>>, std::unordered_map<std::shared_ptr<Edge>, std::vector<Constraint>>> &constraints, double max_time = 0);
    // AStar(std::shared_ptr<AStar> &astar);
    // AStar(std::shared_ptr<Vertex> start, std::shared_ptr<Vertex> goal, std::shared_ptr<Agent> agent, std::vector<Constraint> constraints);
// 
    // void GetPath();
    // double GetEucDistance(std::shared_ptr<Vertex> v1, std::shared_ptr<Vertex> v2);
    // void backtrack(vector<std::shared_ptr<Vertex>> PRMgraph);

private:

    // OpenList<std::tuple<double, unsigned int>, Vertex, hash_tuple::hash<std::tuple<double, unsigned int>>> open_list_h_;

    // the tuple contains: key, id, time
    // I want an open list that contains a tuple of key(tuple of 2), id,time and vertex and a hash function for the tuple
    OpenList<std::tuple<std::tuple<double, double>, unsigned int, double>, Vertex, hash_tuple::hash<std::tuple<std::tuple<double, double>, unsigned int, double>>> open_list_;
    // the tuple contains: id, time
    std::unordered_map<std::tuple<unsigned int, double>, std::shared_ptr<Vertex>, hash_tuple::hash<std::tuple<unsigned int, double>>> closed_list_;
    // the tuple contains: id, time
    std::unordered_map<std::tuple<unsigned int, double>, double, hash_tuple::hash<std::tuple<unsigned int, double>>> g_;
    std::unordered_map<unsigned int, double> h_;
    std::unordered_map<std::tuple<unsigned int, double>, double, hash_tuple::hash<std::tuple<unsigned int, double>>> rhs_;
    double timestep_;
    double key_modifier_;
    std::unordered_map<std::tuple<std::shared_ptr<Vertex>, double>, std::tuple<std::shared_ptr<Vertex>, double>, hash_tuple::hash<std::tuple<std::shared_ptr<Vertex>, double>>> parent_map_;
    double path_time_;
    
    bool isValid(std::shared_ptr<Vertex> vertex, std::shared_ptr<Edge> edge);
    bool isConstrained(std::shared_ptr<Vertex> vertex, std::shared_ptr<Edge> edge, double current_time,
                        std::pair<std::unordered_map<std::shared_ptr<Vertex>, std::vector<Constraint>>, std::unordered_map<std::shared_ptr<Edge>, std::vector<Constraint>>> &constraints);
    // double setRHS(std::shared_ptr<Vertex> vertex, double current_time,
    //               std::pair<std::unordered_map<std::shared_ptr<Vertex>, std::vector<Constraint>>, std::unordered_map<std::shared_ptr<Edge>, std::vector<Constraint>>> &constraints);
    // double getRHS(std::shared_ptr<Vertex> vertex, double current_time,
                //   std::pair<std::unordered_map<std::shared_ptr<Vertex>, std::vector<Constraint>>, std::unordered_map<std::shared_ptr<Edge>, std::vector<Constraint>>> &constraints);
    
    // calculate the key of a vertex
    std::tuple<double, double> calculateKey(std::shared_ptr<Vertex> vertex, double vertex_time);

    // Initialize the search
    void initialize();

    // Update the vertex u
    void updateVertex(std::shared_ptr<Vertex> u, double current_time);
    
};