#pragma once

#include "mamp_planning/mamp_helper.hpp"

using namespace std;
class PRM
{
public:
    //INFO: I am assuming that the start and goal vertices for each agent comes from the agent class
    PRM(std::string planning_group, std::shared_ptr<planning_scene::PlanningScene> planning_scene, double timestep, 
        std::vector<double> &jnt_vel_lim, std::vector<double> &jnt_upper_lim, std::vector<double> &jnt_lower_lim,
        std::shared_ptr<Vertex> start, std::shared_ptr<Vertex> goal);
    std::vector<std::shared_ptr<Vertex>> PRMgraph_;
    std::vector<std::shared_ptr<Vertex>> PRMpath_;
    // std::vector<std::shared_ptr<Vertex>> Astarpath_;



private:
    
    double radius_;
    double epsilon_;
    int num_samples_;
    int dof_;
    bool CheckCollision(std::vector<double> joint_pos);
    double GetDistance(std::shared_ptr<Vertex> v1, std::shared_ptr<Vertex> v2);
    void GetNeighbors(std::shared_ptr<Vertex> q_new, std::vector<std::shared_ptr<Vertex>> graph, double radius);
    std::shared_ptr<Vertex> GetRandomVertex(int dof);
    std::shared_ptr<Vertex> GetNewVertex(std::shared_ptr<Vertex> q_near,std::shared_ptr<Vertex> q, int r);
    std::shared_ptr<Vertex> GetNearestVertex(std::shared_ptr<Vertex> q, std::vector<std::shared_ptr<Vertex>> nodes, int max_id);
    bool Connect(std::shared_ptr<Vertex> q1, std::shared_ptr<Vertex> q2);
    void GetPath(std::shared_ptr<Vertex> q_start, std::shared_ptr<Vertex> q_goal, std::vector<std::shared_ptr<Vertex>> nodes);
    void BuildPRM();


     

    



};