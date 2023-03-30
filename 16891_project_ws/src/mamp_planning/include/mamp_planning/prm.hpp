#pragma once

#include "mamp_planning/mamp_helper.hpp"

using namespace std;
class PRM
{
public:
    //INFO: I am assuming that the start and goal vertices for each agent comes from the agent class
    PRM();
    vector<shared_ptr<Vertex>> PRMgraph_;
    vector<shared_ptr<Vertex>> PRMpath_;
    // vector<shared_ptr<Vertex>> Astarpath_;



private:
    
    double radius_;
    double epsilon_;
    int num_samples_;
    int dof_;
    bool CheckCollision(vector<double> joint_pos);
    double GetDistance(shared_ptr<Vertex> v1, shared_ptr<Vertex> v2);
    void GetNeighbors(shared_ptr<Vertex> q_new, vector<shared_ptr<Vertex>> graph, double radius);
    shared_ptr<Vertex> GetRandomVertex(int dof);
    shared_ptr<Vertex> GetNewVertex(shared_ptr<Vertex> q_near,shared_ptr<Vertex> q, int r);
    shared_ptr<Vertex> GetNearestVertex(shared_ptr<Vertex> q, vector<shared_ptr<Vertex>> nodes, int max_id);
    bool Connect(shared_ptr<Vertex> q1, shared_ptr<Vertex> q2);
    void GetPath(shared_ptr<Vertex> q_start, shared_ptr<Vertex> q_goal, vector<shared_ptr<Vertex>> nodes);
    void BuildPRM();


     

    



};