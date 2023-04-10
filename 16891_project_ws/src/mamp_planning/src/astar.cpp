#include "mamp_planning/astar.hpp"

AStar::AStar(double timestep)
{
  timestep_ = timestep;
  path_time_ = -1;
}

bool AStar::computePRMPath(std::shared_ptr<Vertex> start, std::shared_ptr<Vertex> goal, std::unordered_map<std::shared_ptr<Edge>, Constraint> constraints, double max_constraint_time)
{
  double max_time = computeHeuristics(goal) + max_constraint_time;
  ROS_INFO("h size: %ld", h_.size());
  ROS_INFO("max time: %f", max_time);
  // ROS_INFO("Computed heuristics");
  open_list_.insert(std::make_tuple(h_[start->getId()], start->getId(), 0), start);
  g_.insert({std::make_tuple(start->getId(), 0), 0});
  while (open_list_.size() > 0)
  {
    // ROS_INFO("Inside the while loop");

    auto v = open_list_.pop();
    double g = g_.find(std::make_tuple(v.second->getId(), std::get<2>(v.first)))->second;
    double h = h_[v.second->getId()];
    ROS_INFO("g: %f, h: %f", g, h);
    // ROS_INFO("v id: %d, goal id: %d", v.second->getId(), goal->getId());
    // ROS_INFO("Inside the while loop: after pop");
    double current_time = std::get<2>(v.first);
    ROS_INFO("current time %f", current_time);
    if (v.second->getId() == goal->getId() && current_time > max_constraint_time)
    {
      // ROS_INFO("Yayyyyyy found the goal");
      path_time_ = std::get<2>(v.first);
      return true;
    }
    std::tuple<unsigned int, double> v_closed_tuple = std::make_tuple(v.second->getId(), current_time);
    // ROS_INFO("created v_closed tuple");
    if (closed_list_.insert({v_closed_tuple, v.second}).second && current_time <= max_time)
    {
      // ROS_INFO("Inserted closed list");
      // For staying in same spot
      std::tuple<unsigned int, double> vn_closed_tuple = std::make_tuple(v.second->getId(), current_time + timestep_);
      // ROS_INFO("created vn closed tuple");
      if (closed_list_.find(vn_closed_tuple) == closed_list_.end()) // && !isConstrained(neighbor.second, current_time, constraints))
      {
        // ROS_INFO("Before new_g");
        // ROS_INFO("v closed value %f",v_closed_tuple->second);
        double new_g = g_.find(v_closed_tuple)->second + timestep_;
        // ROS_INFO("Stay in same spot: Got new g");
        std::tuple<double, unsigned int, double> vn_open_tuple = std::make_tuple(new_g + h_.find(v.second->getId())->second,
                                                                                v.second->getId(), 
                                                                                current_time + timestep_);
        // ROS_INFO("Stay in same spot: created vn open tuple");                                                                        
        if (g_.find(vn_closed_tuple) != g_.end())
        {
          // ROS_INFO("Updating the g value");
          double old_g = g_.find(vn_closed_tuple)->second;
          if (old_g > new_g)
          {
            g_.insert({vn_closed_tuple, new_g});
            open_list_.insert(vn_open_tuple, v.second);
            parent_map_.insert({std::make_tuple(v.second, current_time + timestep_), std::make_tuple(v.second, current_time)});
          }
        }
        else
        {
          g_.insert({vn_closed_tuple, new_g});
          open_list_.insert(vn_open_tuple, v.second);
          parent_map_.insert({std::make_tuple(v.second, current_time + timestep_), std::make_tuple(v.second, current_time)});
        }
      }


      // ROS_INFO("Start looking at neighbors");
      // For all neighboring edges
      for (auto neighbor : v.second->getEdges())
      {
        // ROS_INFO("Traversal time %f", neighbor.second->getTraversalTime());
        std::tuple<unsigned int, double> n_closed_tuple = std::make_tuple(neighbor.first->getId(), current_time + neighbor.second->getTraversalTime());
        if (closed_list_.find(n_closed_tuple) == closed_list_.end() &&
            isValid(neighbor.first, neighbor.second) && 
            !isConstrained(neighbor.second, current_time, constraints) &&
            current_time <= max_time)
        {
          // ROS_INFO("Neighbors: inside the first if of the foor loop");
          double new_g = g_.find(v_closed_tuple)->second + neighbor.second->getTraversalTime();
          std::tuple<double, unsigned int, double> n_open_tuple = std::make_tuple(new_g + h_.find(neighbor.first->getId())->second,
                                                                                  neighbor.first->getId(), 
                                                                                  current_time + neighbor.second->getTraversalTime());
          if (g_.find(n_closed_tuple) != g_.end())
          {
            double old_g = g_.find(n_closed_tuple)->second;
            // ROS_INFO("Neighbors: Got old g");
            if (old_g > new_g)
            {
              g_.insert({n_closed_tuple, new_g});
              open_list_.insert(n_open_tuple, neighbor.first);
              // ROS_INFO("Neighbors: overwrote old g");
              parent_map_.insert({std::make_tuple(neighbor.first, current_time + neighbor.second->getTraversalTime()),
                                  std::make_tuple(v.second, current_time)});
            }
          }
          else
          {
            g_.insert({n_closed_tuple, new_g});
            open_list_.insert(n_open_tuple, neighbor.first);
            // ROS_INFO("Neighbors: inserted new g");
            parent_map_.insert({std::make_tuple(neighbor.first, current_time + neighbor.second->getTraversalTime()),
                                std::make_tuple(v.second, current_time)});
          }
        }
      }
    }
    // ROS_INFO("Did not insert into closed list");
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

double AStar::computeHeuristics(std::shared_ptr<Vertex> goal)
{
  OpenList<std::tuple<double, unsigned int>, Vertex, hash_tuple::hash<std::tuple<double, unsigned int>>> open_list_h;
  std::unordered_set<std::shared_ptr<Vertex>> closed_list_h;
  double max_time = 0;
  open_list_h.insert(std::make_tuple(0, goal->getId()), goal);
  // ROS_INFO("goal id %d", goal->getId());
  h_.insert({goal->getId(), 0});

  while (open_list_h.size() != 0)
  {
    std::shared_ptr<Vertex> v = open_list_h.pop().second;
    if (closed_list_h.insert(v).second)
    {
      // ROS_INFO("Inserted vertex into closed list, getting neighbors now" );
      // ROS_INFO("Number of neighbors: %ld", v->getEdges().size());
      for (auto neighbor : v->getEdges())
      {
        // ROS_INFO("closed list_h size %d", closed_list_h.size());
        // ROS_INFO("isValid value %d", isValid(neighbor.first, neighbor.second));
        if (closed_list_h.find(neighbor.first) == closed_list_h.end() &&
            isValid(neighbor.first, neighbor.second))  // Valid edge is not in the open list yet
        {
          max_time += neighbor.second->getTraversalTime();
          // ROS_INFO("Neighbor cost %f", neighbor.second->getTraversalTime());
          if (h_.find(neighbor.first->getId()) != h_.end()) // We have encountered this neighbor before
          {
            // ROS_INFO("Keeping same size of open list");
            double old_h = h_.find(neighbor.first->getId())->second;
            double new_h = h_.find(v->getId())->second + neighbor.second->getTraversalTime();
            if (old_h > new_h)
            {
              h_.insert({neighbor.first->getId(), new_h});
              open_list_h.insert(std::make_tuple(new_h, neighbor.first->getId()), neighbor.first);
            }
          }
          else // Seeing this neighbor for the first time
          {
            // ROS_INFO("Increasing open list");
            double new_h = h_.find(v->getId())->second + neighbor.second->getTraversalTime();
            h_.insert({neighbor.first->getId(), new_h});
            open_list_h.insert(std::make_tuple(new_h, neighbor.first->getId()), neighbor.first);
          }
          // ROS_INFO("open list_h size %d", open_list_h.size());
          
        }
      }
    }
  }
  return max_time;
}

bool AStar::isConstrained(std::shared_ptr<Edge> edge, double current_time, std::unordered_map<std::shared_ptr<Edge>, Constraint> constraints)
{
  auto constraint = constraints.find(edge);
  return constraint != constraints.end() && 
          ((constraint->second.time_step > current_time && constraint->second.time_step < (current_time + edge->getTraversalTime())) ||
          (constraint->second.time_step < -1.0*current_time && constraint->second.time_step > -1.0*(current_time + edge->getTraversalTime())));
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
//                 double g = q->g + q->getEdges()[i]->getTraversalTime();
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
