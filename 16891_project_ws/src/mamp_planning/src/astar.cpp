#include "mamp_planning/astar.hpp"

AStar::AStar(std::shared_ptr<PRM> &prm)
{

}

// AStar::AStar(shared_ptr<Vertex> start, shared_ptr<Vertex> goal, shared_ptr<Agent> agent, std::vector<Constraint> constraints)
// {
//     start_ = start;
//     goal_ = goal;
//     agent_ = agent;
//     constraints_ = constraints;
// }

// double AStar::GetEucDistance(shared_ptr<Vertex> v1, shared_ptr<Vertex> v2)
// {
//     double distance = 0;
//     for (int i = 0; i < v1->joint_pos.size(); i++)
//     {
//         distance += pow(v1->joint_pos[i] - v2->joint_pos[i], 2);
//     }
//     return sqrt(distance);
// }
// void AStar::GetPath()
// {
//     shared_ptr<OpenList> open_list = make_shared<OpenList>();
//     // TODO: sort the open list by f value
   
//     vector<shared_ptr<Vertex>> closed_list;
//     start_->g = 0;
//     start_->h = GetEucDistance(start_, goal_);
//     start_->f = start_->g + start_->h;
//     open_list->insert(start_);
//     unordered_map<int, bool> CLOSED;
//     while (!open_list->empty())
//     {
//         shared_ptr<Vertex> q = open_list->pop();
//         if (q->id == goal_->id)
//         {
//             // cout << "Found the goal!" << endl;
//             break;
//         }

//         // Generate the neighbors of q
//         for(int i = 0; i < q->getEdges().size(); i++)
//         {
//             shared_ptr<Vertex> q_new = q->getEdges()[i]->getVertex();
//             if (CLOSED.find(q_new->id) == CLOSED.end())
//             {
//                 CLOSED[q_new->id] = true;
//                 double g = q->g + q->getEdges()[i]->getCost();
//                 double h = GetEucDistance(q_new, goal_);
//                 double f = g + h;
//                 if (q_new->f == -1 || q_new->f > f)
//                 {
//                     q_new->g = g;
//                     q_new->h = h;
//                     q_new->f = f;
//                     q_new->parent = q;
//                     open_list->insert(q_new);
//                 }
//             }
//         }
//     }
//     backtrack(PRMgraph_);
    
// }

// void AStar::backtrack(vector<shared_ptr<Vertex>> PRMgraph)
// {
//     shared_ptr<Vertex> q = goal_;
//     while (q->id != start_->id)
//     {
//         Astarpath_.push_back(q);
//         q = q->parent;
//     }
//     Astarpath_.push_back(start_);
//     reverse(Astarpath_.begin(), Astarpath_.end());
// }

        





