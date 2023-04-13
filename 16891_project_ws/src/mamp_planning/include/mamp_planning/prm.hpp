#pragma once

#include "mamp_planning/mamp_helper.hpp"

using namespace std;
class PRM
{
public:
    //INFO: I am assuming that the start and goal vertices for each agent comes from the agent class
    PRM(std::shared_ptr<planning_scene::PlanningScene> planning_scene, double timestep, 
        std::vector<double> &jnt_vel_lim, std::vector<double> &jnt_upper_lim, std::vector<double> &jnt_lower_lim,
        std::shared_ptr<Vertex> start, std::shared_ptr<Vertex> goal);
    std::vector<std::shared_ptr<Vertex>> PRMgraph_;
    std::vector<std::shared_ptr<Vertex>> PRMpath_;
    // std::vector<std::shared_ptr<Vertex>> Astarpath_;
    void getPath(std::vector<std::shared_ptr<Vertex>> nodes);
    void buildPRM();
    void expandPRM();



private:
    
    double radius_;
    double epsilon_;
    int num_samples_;
    int dof_;
    int node_id_;
    unsigned int component_;
    double expansion_factor_;
    std::shared_ptr<Vertex> start_;
    std::shared_ptr<Vertex> goal_;
    std::shared_ptr<planning_scene::PlanningScene> planning_scene_;
    std::vector<double> jnt_vel_lim_;
    std::vector<double> jnt_upper_lim_;
    std::vector<double> jnt_lower_lim_;
    double timestep_;
    bool checkCollision(std::vector<double> joint_pos);
    double getDistance(std::shared_ptr<Vertex> v1, std::shared_ptr<Vertex> v2);
    void getNeighbors(std::shared_ptr<Vertex> q_new, double radius);
    std::shared_ptr<Vertex> getRandomVertex();
    std::shared_ptr<Vertex> getNewVertex(std::shared_ptr<Vertex> q_near,std::shared_ptr<Vertex> q, int r);
    std::shared_ptr<Vertex> getNearestVertex(std::shared_ptr<Vertex> q, std::vector<std::shared_ptr<Vertex>> nodes, int max_id);
    bool connect(std::shared_ptr<Vertex> q1, std::shared_ptr<Vertex> q2);


     

    



};