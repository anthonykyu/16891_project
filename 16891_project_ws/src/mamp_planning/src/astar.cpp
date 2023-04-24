#include "mamp_planning/astar.hpp"

AStar::AStar(double timestep)
{
  timestep_ = timestep;
  path_time_ = 0;
}

bool AStar::computeWaypointPaths(std::vector<std::shared_ptr<Vertex>> waypoints,
                        std::pair<std::unordered_map<std::shared_ptr<Vertex>, std::vector<Constraint>>, std::unordered_map<std::shared_ptr<Edge>, std::vector<Constraint>>> &constraints,
                        double max_constraint_time)
{
  bool success = true;
  path_time_ = 0;
  for (int i = 0; i < waypoints.size()-1; ++i)
  {
    if (i == waypoints.size()-2) 
      success = computePRMPath(waypoints[i], waypoints[i+1], constraints, max_constraint_time, path_time_, true);
    else
      success = computePRMPath(waypoints[i], waypoints[i+1], constraints, max_constraint_time, path_time_, false);
  }
  return success;
}

bool AStar::computePRMPath(std::shared_ptr<Vertex> start, std::shared_ptr<Vertex> goal, 
std::pair<std::unordered_map<std::shared_ptr<Vertex>, std::vector<Constraint>>, std::unordered_map<std::shared_ptr<Edge>, std::vector<Constraint>>> &constraints,
double max_constraint_time, double start_time, bool is_last_waypoint)
{
  // ROS_INFO("Running an episode of Astar search");
  open_list_.clear();
  closed_list_.clear();
  // parent_map_.clear();
  g_.clear();

  double max_time = computeHeuristics(goal) + max_constraint_time;
  // ROS_INFO("h size: %ld", h_.size());
  // ROS_INFO("max time: %f", max_time);
  // ROS_INFO("Computed heuristics");
  open_list_.insert(std::make_tuple(h_[start->getId()], start->getId(), start_time), std::make_tuple(start->getId(), round(start_time)), start);
  g_.insert({std::make_tuple(start->getId(), start_time), 0});
  while (open_list_.size() > 0)
  {
    // ROS_INFO("Inside the while loop");

    auto v = open_list_.pop();
    std::tuple<double, unsigned int, double> comparison_v = std::get<0>(v);
    std::tuple<unsigned int, double> id_v = std::get<1>(v); 
    std::shared_ptr<Vertex> vertex_v = std::get<2>(v); 
    double g = g_.find(id_v)->second;
    double h = h_[vertex_v->getId()];
    // ROS_INFO("g: %f, h: %f", g, h);
    // ROS_INFO("v id: %d, goal id: %d", v.second->getId(), goal->getId());
    // ROS_INFO("Inside the while loop: after pop");
    double current_time = std::get<1>(id_v);
    // ROS_INFO("current time %f", current_time);
    if (vertex_v->getId() == goal->getId() && (current_time > max_constraint_time || !is_last_waypoint))
    {
      // ROS_INFO("Yayyyyyy found the goal");
      path_time_ = current_time;
      return true;
    }
    std::tuple<unsigned int, double> v_closed_tuple = std::make_tuple(vertex_v->getId(), round(current_time));
    // ROS_INFO("created v_closed tuple");
    if (closed_list_.insert({v_closed_tuple, vertex_v}).second && current_time <= max_time)
    {
      // ROS_INFO("Inserted closed list");
      // For staying in same spot
      std::tuple<unsigned int, double> vn_closed_tuple = std::make_tuple(vertex_v->getId(), round(current_time + timestep_));
      // ROS_INFO("created vn closed tuple");
      if (closed_list_.find(vn_closed_tuple) == closed_list_.end() && !isConstrained(vertex_v ,nullptr, current_time, constraints))
      {
        // ROS_INFO("Before new_g");
        // ROS_INFO("v closed value %f",v_closed_tuple->second);
        double new_g = g_.find(v_closed_tuple)->second + timestep_;
        // ROS_INFO("Stay in same spot: Got new g");
        std::tuple<double, unsigned int, double> vn_open_tuple = std::make_tuple(new_g + h_.find(vertex_v->getId())->second,
                                                                                vertex_v->getId(), 
                                                                                current_time + timestep_);
        // ROS_INFO("Stay in same spot: created vn open tuple");                                                                        
        if (g_.find(vn_closed_tuple) != g_.end())
        {
          // ROS_INFO("Updating the g value");
          double old_g = g_.find(vn_closed_tuple)->second;
          if (old_g > new_g)
          {
            g_.erase(vn_closed_tuple);
            g_.insert({vn_closed_tuple, new_g});
            open_list_.insert(vn_open_tuple, vn_closed_tuple, vertex_v);
            parent_map_.erase(std::make_tuple(vertex_v, round(current_time + timestep_)));
            parent_map_.insert({std::make_tuple(vertex_v, round(current_time + timestep_)), std::make_tuple(vertex_v, round(current_time))});
          }
        }
        else
        {
          g_.insert({vn_closed_tuple, new_g});
          open_list_.insert(vn_open_tuple, vn_closed_tuple, vertex_v);
          parent_map_.erase(std::make_tuple(vertex_v, round(current_time + timestep_)));
          parent_map_.insert({std::make_tuple(vertex_v, round(current_time + timestep_)), std::make_tuple(vertex_v, round(current_time))});
        }
      }


      // ROS_INFO("Start looking at neighbors");
      // For all neighboring edges
      for (auto neighbor : vertex_v->getEdges())
      {
        // ROS_INFO("Traversal time %f", neighbor.second->getTraversalTime());
        std::tuple<unsigned int, double> n_closed_tuple = std::make_tuple(neighbor.first->getId(), round(current_time + neighbor.second->getTraversalTime()));
        if (closed_list_.find(n_closed_tuple) == closed_list_.end() &&
            isValid(neighbor.first, neighbor.second) && 
            !isConstrained(neighbor.first, neighbor.second, current_time, constraints) &&
            current_time <= max_time)
        {
          // ROS_INFO("Neighbors: inside the first if of the foor loop");
          // ROS_INFO("neighbor.second %d", neighbor.second != nullptr);
          // ROS_INFO("neighbor.second->getTraversalTime() %f", neighbor.second->getTraversalTime());
          double new_g = g_.find(v_closed_tuple)->second + neighbor.second->getTraversalTime();
          // ROS_INFO("neighbor.second->getTraversalTime() %f", neighbor.second->getTraversalTime());
          // ROS_INFO("neighbor.first->getId()->second %d", neighbor.first->getId());
          // ROS_INFO("current_time %f", current_time);
          // ROS_INFO("h_.find(neighbor.first->getId()) %f", h_.find(neighbor.first->getId())->first);

          // ROS_INFO("neighbor.first->getId() %d", neighbor.first->getId());
          // ROS_INFO("h_.find(neighbor.first->getId()) %d", h_.find(neighbor.first->getId()) == h_.end());
          
          // ROS_INFO("h_.find(neighbor.first->getId())->first %d", h_.find(neighbor.first->getId())->first);
          // ROS_INFO("h_.find(neighbor.first->getId())->second %f", h_.find(neighbor.first->getId())->second);
          std::tuple<double, unsigned int, double> n_open_tuple = std::make_tuple(new_g + h_.find(neighbor.first->getId())->second,
                                                                                  neighbor.first->getId(), 
                                                                                  current_time + neighbor.second->getTraversalTime());
          // ROS_INFO("Neighbors: about to calc g and insert");
          if (g_.find(n_closed_tuple) != g_.end())
          {
            double old_g = g_.find(n_closed_tuple)->second;
            // ROS_INFO("Neighbors: Got old g");
            if (old_g > new_g)
            {
              g_.erase(n_closed_tuple);
              g_.insert({n_closed_tuple, new_g});
              open_list_.insert(n_open_tuple, n_closed_tuple, neighbor.first);
              // ROS_INFO("Neighbors: overwrote old g");
              parent_map_.erase(std::make_tuple(neighbor.first, round(current_time + neighbor.second->getTraversalTime())));
              parent_map_.insert({std::make_tuple(neighbor.first, round(current_time + neighbor.second->getTraversalTime())),
                                  std::make_tuple(vertex_v, round(current_time))});
            }
          }
          else
          {
            g_.insert({n_closed_tuple, new_g});
            open_list_.insert(n_open_tuple, n_closed_tuple, neighbor.first);
            // ROS_INFO("Neighbors: inserted new g");
            parent_map_.erase(std::make_tuple(neighbor.first, round(current_time + neighbor.second->getTraversalTime())));
            parent_map_.insert({std::make_tuple(neighbor.first, round(current_time + neighbor.second->getTraversalTime())),
                                std::make_tuple(vertex_v, round(current_time))});
          }
        }
        // else
        // {
        //   ROS_INFO("SKIPPED");
        // }
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
  std::tuple<std::shared_ptr<Vertex>, double> v_time = std::make_tuple(goal, round(path_time_));
  prm_path.push_back(v);
  double time = path_time_;
  while (v != start || time != 0)
  {
    v_time = parent_map_.find(v_time)->second;
    v = std::get<0>(v_time);
    time = std::get<1>(v_time);
    prm_path.push_back(v);
  }
  std::reverse(prm_path.begin(), prm_path.end());
  // ROS_INFO("Returning a path from Astar");
  return prm_path;
}

double AStar::computeHeuristics(std::shared_ptr<Vertex> goal)
{
  // ROS_INFO("Computing heuristics");
  h_.clear();
  OpenList<std::tuple<double, unsigned int>, std::tuple<unsigned int>, Vertex, hash_tuple::hash<std::tuple<unsigned int>>> open_list_h;
  std::unordered_set<std::shared_ptr<Vertex>> closed_list_h;
  std::unordered_set<std::shared_ptr<Edge>> closed_edge_list;
  double max_time = 0;
  open_list_h.insert(std::make_tuple(0, goal->getId()), std::make_tuple(goal->getId()), goal);
  // ROS_INFO("goal id %d", goal->getId());
  h_.insert({goal->getId(), 0});

  while (open_list_h.size() != 0)
  {
    std::shared_ptr<Vertex> v = std::get<2>(open_list_h.pop());
    // ROS_INFO("open list size: %ld", open_list_h.size());
    // ROS_INFO("v is null? %d", v == nullptr);
    if (closed_list_h.insert(v).second)
    {
      // ROS_INFO("Inserted vertex into closed list, getting neighbors now" );
      // ROS_INFO("Number of neighbors: %ld", v->getEdges().size());
      // v->getEdges();
      for (auto neighbor : v->getEdges())
      {
        // ROS_INFO("Inserted vertex into closed list, getting neighbors now" );
        // ROS_INFO("closed list_h size %ld", closed_list_h.size());
        // ROS_INFO("isValid value %d", isValid(neighbor.first, neighbor.second));
        if (closed_list_h.find(neighbor.first) == closed_list_h.end() &&
            isValid(neighbor.first, neighbor.second))  // Valid edge is not in the open list yet
        {
          if (closed_edge_list.find(neighbor.second) == closed_edge_list.end())
          {
            max_time += neighbor.second->getTraversalTime();
            closed_edge_list.insert(neighbor.second);
          }
          // ROS_INFO("Neighbor cost %f", neighbor.second->getTraversalTime());
          if (h_.find(neighbor.first->getId()) != h_.end()) // We have encountered this neighbor before
          {
            // ROS_INFO("Keeping same size of open list");
            double old_h = h_.find(neighbor.first->getId())->second;
            double new_h = h_.find(v->getId())->second + neighbor.second->getTraversalTime();
            if (old_h > new_h)
            {
              h_.erase(neighbor.first->getId());
              h_.insert({neighbor.first->getId(), new_h});
              // ROS_INFO("Increasing open list 1");
              open_list_h.insert(std::make_tuple(new_h, neighbor.first->getId()), std::make_tuple(neighbor.first->getId()), neighbor.first);
            }
          }
          else // Seeing this neighbor for the first time
          {
            double new_h = h_.find(v->getId())->second + neighbor.second->getTraversalTime();
            h_.insert({neighbor.first->getId(), new_h});
            // ROS_INFO("Increasing open list");
            open_list_h.insert(std::make_tuple(new_h, neighbor.first->getId()), std::make_tuple(neighbor.first->getId()), neighbor.first);
          }
          // ROS_INFO("open list_h size %d", open_list_h.size());
          
        }
      }
      // ROS_INFO("After for loop" );
    }
  }
  // ROS_WARN("Done with heuristics");
  return max_time;
}

bool AStar::isConstrained(std::shared_ptr<Vertex> vertex, std::shared_ptr<Edge> edge, double current_time,
std::pair<std::unordered_map<std::shared_ptr<Vertex>, std::vector<Constraint>>, std::unordered_map<std::shared_ptr<Edge>, std::vector<Constraint>>> &constraints)
{
  auto constraint = constraints.second.find(edge);
  auto constrained_vertex1 = constraints.first.find(vertex);

  if (edge)
  {
    bool edge_constrained = constraint != constraints.second.end();
    if (edge_constrained)
    {
      for (auto c : constraint->second)
      {
        edge_constrained = ((round(c.time_step) >= round(current_time) && round(c.time_step) <= round(current_time + edge->getTraversalTime())) ||
                            (round(c.time_step) <= -1.0 * round(current_time) && round(c.time_step) >= -1.0 * round(current_time + edge->getTraversalTime())));
        if (edge_constrained)
        {
          // ROS_INFO("IS EDGE CONSTRAINED");
          return true;
        }
      }
    }

    bool vertex_constrained1 = constrained_vertex1 != constraints.first.end();
    if (vertex_constrained1)
    {
      for (auto c : constrained_vertex1->second)
      {
        vertex_constrained1 = (c.time_step >= (current_time + edge->getTraversalTime() - (timestep_ / 2.0)) && c.time_step <= (current_time + edge->getTraversalTime() + (timestep_ / 2.0))) ||
                             (c.time_step <= -1.0 * (current_time + edge->getTraversalTime() - timestep_ / 2.0) && c.time_step >= -1.0 * (current_time + edge->getTraversalTime() + timestep_ / 2.0));
        if (vertex_constrained1)
        {
          // ROS_INFO("IS VERTEX1 CONSTRAINED, c.timestep: %f, current_time: %f, trav_time: %f", c.time_step, current_time, edge->getTraversalTime());
          return true;
        }
      }
    }

    auto constrained_vertex2 = constraints.first.find(edge->getOpposingVertex(vertex));
    bool vertex_constrained2 = constrained_vertex2 != constraints.first.end();
    if (vertex_constrained2)
    {
      for (auto c : constrained_vertex2->second)
      {
        vertex_constrained2 = (c.time_step >= (current_time - (timestep_ / 2.0)) && c.time_step <= (current_time + (timestep_ / 2.0))) ||
                             (c.time_step <= -1.0 * (current_time - timestep_ / 2.0) && c.time_step >= -1.0 * (current_time + timestep_ / 2.0));
        if (vertex_constrained2)
        {
          // ROS_INFO("IS VERTEX1 CONSTRAINED, c.timestep: %f, current_time: %f, trav_time: %f", c.time_step, current_time, edge->getTraversalTime());
          return true;
        }
      }
    }
  }
  else
  {
    bool vertex_constrained1 = constrained_vertex1 != constraints.first.end();
    if (vertex_constrained1)
    {
      for (auto c : constrained_vertex1->second)
      {
        vertex_constrained1 = (c.time_step >= (current_time + (0.5 * timestep_)) && c.time_step <= (current_time + (1.5 * timestep_))) ||
                             (c.time_step <= -1.0 * (current_time + 0.5 * timestep_) && c.time_step >= -1.0 * (current_time + 1.5 * timestep_));
        vertex_constrained1 = vertex_constrained1 || (c.time_step >= (current_time - (0.5 * timestep_)) && c.time_step <= (current_time + (0.5 * timestep_))) ||
                                                    (c.time_step <= -1.0 * (current_time - 0.5 * timestep_) && c.time_step >= -1.0 * (current_time + 0.5 * timestep_));
        if (vertex_constrained1)
        {
          // ROS_INFO("IS VERTEX2 CONSTRAINED");
          return true;
        }
      }
    }
  }
  return false;
}

bool AStar::isValid(std::shared_ptr<Vertex> vertex, std::shared_ptr<Edge> edge)
{
  return edge->isValid() && vertex->isValid();;
}

double AStar::round(double in)
{
  return static_cast<int>(std::round(in / timestep_)) * timestep_;
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
