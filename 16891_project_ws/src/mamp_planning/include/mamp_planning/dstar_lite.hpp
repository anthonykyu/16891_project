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
    // std::vector<std::shared_ptr<Vertex>> Astarpath_;

    
    DStarLite(std::shared_ptr<PRM> prm, double timestep);
    std::shared_ptr<Vertex> getParent();
    // AStar(std::shared_ptr<PRM> &prm);
    double computeHeuristics(std::shared_ptr<Vertex> start);
    // void computePRMPath(std::shared_ptr<Vertex> start, std::shared_ptr<Vertex> goal,
    //                     std::pair<std::unordered_map<std::shared_ptr<Vertex>, std::vector<Constraint>>, std::unordered_map<std::shared_ptr<Edge>, std::vector<Constraint>>> &constraints,
    //                     double max_constraint_time);

    // TODO: need to add the constraints
    // void computePRMPath(std::shared_ptr<Vertex> start, std::shared_ptr<Vertex> goal,
    //                     double max_constraint_time);
    void computePRMPath(std::shared_ptr<Vertex> start, std::shared_ptr<Vertex> goal,
                        std::pair<std::unordered_map<std::shared_ptr<Vertex>, std::vector<Constraint>>, std::unordered_map<std::shared_ptr<Edge>, std::vector<Constraint>>> &constraints);
    std::vector<std::shared_ptr<Vertex>> getPRMPath(std::vector<std::shared_ptr<Vertex>> Astarpath, std::shared_ptr<Vertex> start, std::shared_ptr<Vertex> goal, std::pair<std::unordered_map<std::shared_ptr<Vertex>, std::vector<Constraint>>, std::unordered_map<std::shared_ptr<Edge>, std::vector<Constraint>>> &constraints, double max_time = 0);
    // AStar(std::shared_ptr<AStar> &astar);
    // AStar(std::shared_ptr<Vertex> start, std::shared_ptr<Vertex> goal, std::shared_ptr<Agent> agent, std::vector<Constraint> constraints);
// 
    // void GetPath();
    // double GetEucDistance(std::shared_ptr<Vertex> v1, std::shared_ptr<Vertex> v2);
    // void backtrack(vector<std::shared_ptr<Vertex>> PRMgraph);

private:

    // OpenList<std::tuple<double, unsigned int>, Vertex, hash_tuple::hash<std::tuple<double, unsigned int>>> open_list_h_;

    // the tuple contains: key, id, time
    // I want an open list that contains a tuple of key(tuple of 2), id and vertex and a hash function for the tuple
    OpenList<std::tuple<double, double, unsigned int>, std::tuple<unsigned int>, Vertex, hash_tuple::hash<std::tuple<unsigned int>>> open_list_;
    std::unordered_map<unsigned int, std::shared_ptr<Vertex>>  closed_list_;
    std::unordered_map<unsigned int, double> g_;
    std::unordered_map<unsigned int, double> h_;
    std::unordered_map<unsigned int, double> rhs_;
    std::unordered_map<std::shared_ptr<Edge>, double> changed_edges_;
    std::shared_ptr<PRM> prm_;
    double timestep_;
    double key_modifier_;
    // std::unordered_map<std::shared_ptr<Vertex>>, std::shared_ptr<Vertex>> parent_map_;
    double path_time_;
    
    double getEdgeCost(std::shared_ptr<Edge> edge);
    bool isValid(std::shared_ptr<Vertex> vertex, std::shared_ptr<Edge> edge);
    bool isConstrained(std::shared_ptr<Vertex> vertex, std::shared_ptr<Edge> edge, double current_time,
                        std::pair<std::unordered_map<std::shared_ptr<Vertex>, std::vector<Constraint>>, std::unordered_map<std::shared_ptr<Edge>, std::vector<Constraint>>> &constraints);
    // double setRHS(std::shared_ptr<Vertex> vertex, double current_time,
    //               std::pair<std::unordered_map<std::shared_ptr<Vertex>, std::vector<Constraint>>, std::unordered_map<std::shared_ptr<Edge>, std::vector<Constraint>>> &constraints);
    // double getRHS(std::shared_ptr<Vertex> vertex, double current_time,
                //   std::pair<std::unordered_map<std::shared_ptr<Vertex>, std::vector<Constraint>>, std::unordered_map<std::shared_ptr<Edge>, std::vector<Constraint>>> &constraints);
    
    // calculate the key of a vertex
    std::tuple<double, double> calculateKey(std::shared_ptr<Vertex> vertex);

    // Initialize the search
    void initialize();

    // Update the vertex u
    void updateVertex(std::shared_ptr<Vertex> u);

    bool compareKeys(std::tuple<double, double> key1, std::tuple<double, double> key2);
    double getTraversalTimeBetweenVertices(std::vector<std::shared_ptr<Vertex>> vertices, int idx_1, int idx_2);
};