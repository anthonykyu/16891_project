#include "mamp_planning/astar.hpp"

AStar::AStar(double timestep)
{
  timestep_ = timestep;
  path_time_ = -1;
}

bool AStar::computePRMPath(std::shared_ptr<Vertex> start, std::shared_ptr<Vertex> goal, std::unordered_map<std::shared_ptr<Edge>, Constraint> constraints)
{
  open_list_.insert(std::make_tuple(h_[start->getId()], start->getId(), 0), start);
  while (open_list_.size() > 0)
  {
    auto v = open_list_.pop();
    if (v.second == goal)
    {
      path_time_ = std::get<2>(v.first);
      return true;
    }

    double current_time = std::get<2>(v.first);
    std::tuple<unsigned int, double> v_closed_tuple = std::make_tuple(v.second->getId(), current_time);
    if (closed_list_.insert({v_closed_tuple, v.second}).second)
    {
      // For staying in same spot
      std::tuple<unsigned int, double> vn_closed_tuple = std::make_tuple(v.second->getId(), current_time + timestep_);
      if (closed_list_.find(vn_closed_tuple) == closed_list_.end()) // && !isConstrained(neighbor.second, current_time, constraints))
      {
        double new_g = g_.find(vn_closed_tuple)->second + timestep_;
        std::tuple<double, unsigned int, double> vn_open_tuple = std::make_tuple(new_g + h_.find(v.second->getId())->second,
                                                                                v.second->getId(), 
                                                                                current_time + timestep_);
        if (g_.find(vn_closed_tuple) != g_.end())
        {
          double old_g = g_.find(vn_closed_tuple)->second;
          if (old_g > new_g)
          {
            g_.insert({vn_closed_tuple, new_g});
            open_list_.insert(vn_open_tuple, v.second);
            // v.second->setParent(v.second); // TODO: This has to change!!!
            parent_map_.insert({std::make_tuple(v.second, current_time + timestep_), std::make_tuple(v.second, current_time)});
          }
        }
        else
        {
          g_.insert({vn_closed_tuple, new_g});
          open_list_.insert(vn_open_tuple, v.second);
          // v.second->setParent(v.second); // TODO: This has to change!!!
          parent_map_.insert({std::make_tuple(v.second, current_time + timestep_), std::make_tuple(v.second, current_time)});
        }
      }

      // For all neighboring edges
      for (auto neighbor : v.second->getEdges())
      {
        std::tuple<unsigned int, double> n_closed_tuple = std::make_tuple(neighbor.first->getId(), current_time + neighbor.second->getTraversalTime());
        if (closed_list_.find(n_closed_tuple) == closed_list_.end() &&
            isValid(neighbor.first, neighbor.second) && !isConstrained(neighbor.second, current_time, constraints))
        {
          double new_g = g_.find(v_closed_tuple)->second + neighbor.second->getCost() + timestep_;
          std::tuple<double, unsigned int, double> n_open_tuple = std::make_tuple(new_g + h_.find(neighbor.first->getId())->second,
                                                                                  neighbor.first->getId(), 
                                                                                  current_time + neighbor.second->getTraversalTime());
          if (g_.find(n_closed_tuple) != g_.end())
          {
            double old_g = g_.find(n_closed_tuple)->second;
            if (old_g > new_g)
            {
              g_.insert({n_closed_tuple, new_g});
              open_list_.insert(n_open_tuple, neighbor.first);
              // neighbor.first->setParent(v.second);
              parent_map_.insert({std::make_tuple(neighbor.first, current_time + neighbor.second->getTraversalTime()),
                                  std::make_tuple(v.second, current_time)});
            }
          }
          else
          {
            g_.insert({n_closed_tuple, new_g});
            open_list_.insert(n_open_tuple, neighbor.first);
            // neighbor.first->setParent(v.second);
            parent_map_.insert({std::make_tuple(neighbor.first, current_time + neighbor.second->getTraversalTime()),
                                std::make_tuple(v.second, current_time)});
          }
        }
      }
    }
  }
  return false;
}

std::vector<std::shared_ptr<Vertex>> AStar::getPRMPath(std::shared_ptr<Vertex> start, std::shared_ptr<Vertex> goal)
{
  std::vector<std::shared_ptr<Vertex>> prm_path;
  std::shared_ptr<Vertex> v = goal;
  std::tuple<std::shared_ptr<Vertex>, double> v_time = std::make_tuple(goal, path_time_);
  prm_path.push_back(v);
  while (v != start)
  {
    v_time = parent_map_.find(v_time)->second;
    v = std::get<0>(v_time);
    prm_path.push_back(v);
  }
  std::reverse(prm_path.begin(), prm_path.end());
  return prm_path;
}

void AStar::computeHeuristics(std::shared_ptr<Vertex> goal)
{
  OpenList<std::tuple<double, unsigned int>, Vertex, hash_tuple::hash<std::tuple<double, unsigned int>>> open_list_h;
  std::unordered_set<std::shared_ptr<Vertex>> closed_list_h;

  open_list_h.insert(std::make_tuple(0, goal->getId()), goal);
  h_.insert({goal->getId(), 0});

  while (open_list_h.size() != 0)
  {
    std::shared_ptr<Vertex> v = open_list_h.pop().second;
    if (closed_list_h.insert(v).second)
    {
      for (auto neighbor : v->getEdges())
      {
        if (closed_list_h.find(neighbor.first) == closed_list_h.end() &&
            isValid(neighbor.first, neighbor.second))
        {
          if (h_.find(neighbor.first->getId()) != h_.end())
          {
            double old_h = h_.find(neighbor.first->getId())->second;
            double new_h = h_.find(v->getId())->second + neighbor.second->getCost();
            if (old_h > new_h)
            {
              h_.insert({neighbor.first->getId(), new_h});
              open_list_h.insert(std::make_tuple(new_h, neighbor.first->getId()), neighbor.first);
            }
          }
          else
          {
            double new_h = h_.find(v->getId())->second + neighbor.second->getCost();
            h_.insert({neighbor.first->getId(), new_h});
            open_list_h.insert(std::make_tuple(new_h, neighbor.first->getId()), neighbor.first);
          }
        }
      }
    }
  }

}

bool AStar::isConstrained(std::shared_ptr<Edge> edge, double current_time, std::unordered_map<std::shared_ptr<Edge>, Constraint> constraints)
{
  auto constraint = constraints.find(edge);
  return constraint != constraints.end() && constraint->second.time_step > current_time && constraint->second.time_step < (current_time + edge->getTraversalTime());
}

bool AStar::isValid(std::shared_ptr<Vertex> vertex, std::shared_ptr<Edge> edge)
{
  return edge->isValid() && vertex->isValid();;
}


// AStar::AStar(std::shared_ptr<PRM> &prm)
// {
//   prm_ = prm;
// }

// AStar::AStar(std::shared_ptr<AStar> &astar)
// {

// }

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
