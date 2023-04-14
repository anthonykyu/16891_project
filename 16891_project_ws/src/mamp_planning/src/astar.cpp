#include "mamp_planning/astar.hpp"

AStar::AStar(double timestep)
{
  timestep_ = timestep;
  path_time_ = -1;
}

bool AStar::computePRMPath(std::shared_ptr<Vertex> start, std::shared_ptr<Vertex> goal, 
std::pair<std::unordered_map<std::shared_ptr<Vertex>, std::vector<Constraint>>, std::unordered_map<std::shared_ptr<Edge>, std::vector<Constraint>>> &constraints,
double max_constraint_time)
{
  // ROS_INFO("Running an episode of Astar search");
  open_list_.clear();
  closed_list_.clear();
  parent_map_.clear();
  g_.clear();
  h_.clear();

  double max_time = computeHeuristics(goal) + max_constraint_time;
  // ROS_INFO("h size: %ld", h_.size());
  // ROS_INFO("max time: %f", max_time);
  // ROS_INFO("Computed heuristics");
  open_list_.insert(std::make_tuple(h_[start->getId()], start->getId(), 0), start);
  g_.insert({std::make_tuple(start->getId(), 0), 0});
  while (open_list_.size() > 0)
  {
    // ROS_INFO("Inside the while loop");

    auto v = open_list_.pop();
    double g = g_.find(std::make_tuple(v.second->getId(), std::get<2>(v.first)))->second;
    double h = h_[v.second->getId()];
    // ROS_INFO("g: %f, h: %f", g, h);
    // ROS_INFO("v id: %d, goal id: %d", v.second->getId(), goal->getId());
    // ROS_INFO("Inside the while loop: after pop");
    double current_time = std::get<2>(v.first);
    // ROS_INFO("current time %f", current_time);
    if (v.second->getId() == goal->getId() && current_time > max_constraint_time)
    {
      // ROS_INFO("Yayyyyyy found the goal");
      path_time_ = current_time;
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
      if (closed_list_.find(vn_closed_tuple) == closed_list_.end() && !isConstrained(v.second ,nullptr, current_time, constraints))
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
            g_.erase(vn_closed_tuple);
            g_.insert({vn_closed_tuple, new_g});
            open_list_.insert(vn_open_tuple, v.second);
            parent_map_.erase(std::make_tuple(v.second, current_time + timestep_));
            parent_map_.insert({std::make_tuple(v.second, current_time + timestep_), std::make_tuple(v.second, current_time)});
          }
        }
        else
        {
          g_.insert({vn_closed_tuple, new_g});
          open_list_.insert(vn_open_tuple, v.second);
          parent_map_.erase(std::make_tuple(v.second, current_time + timestep_));
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
              open_list_.insert(n_open_tuple, neighbor.first);
              // ROS_INFO("Neighbors: overwrote old g");
              parent_map_.erase(std::make_tuple(neighbor.first, current_time + neighbor.second->getTraversalTime()));
              parent_map_.insert({std::make_tuple(neighbor.first, current_time + neighbor.second->getTraversalTime()),
                                  std::make_tuple(v.second, current_time)});
            }
          }
          else
          {
            g_.insert({n_closed_tuple, new_g});
            open_list_.insert(n_open_tuple, neighbor.first);
            // ROS_INFO("Neighbors: inserted new g");
            parent_map_.erase(std::make_tuple(neighbor.first, current_time + neighbor.second->getTraversalTime()));
            parent_map_.insert({std::make_tuple(neighbor.first, current_time + neighbor.second->getTraversalTime()),
                                std::make_tuple(v.second, current_time)});
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
  std::tuple<std::shared_ptr<Vertex>, double> v_time = std::make_tuple(goal, path_time_);
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
  OpenList<std::tuple<double, unsigned int>, Vertex, hash_tuple::hash<std::tuple<double, unsigned int>>> open_list_h;
  std::unordered_set<std::shared_ptr<Vertex>> closed_list_h;
  std::unordered_set<std::shared_ptr<Edge>> closed_edge_list;
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

bool AStar::isConstrained(std::shared_ptr<Vertex> vertex, std::shared_ptr<Edge> edge, double current_time,
std::pair<std::unordered_map<std::shared_ptr<Vertex>, std::vector<Constraint>>, std::unordered_map<std::shared_ptr<Edge>, std::vector<Constraint>>> &constraints)
{
  auto constraint = constraints.second.find(edge);
  auto constrained_vertex = constraints.first.find(vertex);
  

  if (edge)
  {
    bool edge_constrained = constraint != constraints.second.end();
    if (edge_constrained)
    {
      for (auto c : constraint->second)
      {
        edge_constrained = ((c.time_step >= current_time && c.time_step <= (current_time + edge->getTraversalTime())) ||
                          (c.time_step <= -1.0*current_time && c.time_step >= -1.0*(current_time + edge->getTraversalTime())));
        if (edge_constrained)
          return true;
      }
    }
     
    bool vertex_constrained = constrained_vertex != constraints.first.end();
    if (vertex_constrained)
    {
      for (auto c : constrained_vertex->second)
      {
        // ROS_INFO("Vertex Constraint: Agent: %s, Time: %f", c.agent_id.c_str(), c.time_step);
        // ROS_INFO("Neighbor Time: %f", current_time + edge->getTraversalTime());
        // ROS_INFO("Equality check: %d", c.time_step >= (current_time + edge->getTraversalTime() - timestep_) && c.time_step <= (current_time + edge->getTraversalTime() + timestep_));
        // ROS_INFO("Neighbor Vertex: ");
        // for (double j : vertex->getJointPos())
        // {
        //   ROS_INFO("%f, ", j);
        // }
        // ROS_INFO("Constraint Vertex: ");
        // for (double j : c.joint_pos_vertex->getJointPos())
        // {
        //   ROS_INFO("%f, ", j);
        // }
        vertex_constrained = (c.time_step >= (current_time + edge->getTraversalTime() - timestep_) && c.time_step <= (current_time + edge->getTraversalTime() + timestep_)) ||
                            (c.time_step <= -1.0*(current_time + edge->getTraversalTime() - timestep_) && c.time_step >= -1.0*(current_time + edge->getTraversalTime() + timestep_));
        // ROS_INFO("Vertex Constrained: %d", vertex_constrained);
        if (vertex_constrained)
          return true;
      }
    }
          
    // if (edge_constrained)
    // {
    //   ROS_INFO("Edge Constrained!!!");
    //   // ROS_INFO("%s, %d, %f", constraint->second.agent_id.c_str(), constraint->second.is_vertex_constraint, constraint->second.time_step);

    //   ROS_INFO("Trying to travel to Vertex at time %f with travel time %f and constraint time %f: ", current_time, edge->getTraversalTime(), constraint->second.time_step);
    //   // for (double j : vertex->getJointPos())
    //   // {
    //   //   ROS_INFO("%f, ", j);
    //   // }

    //   if (constraint->second.is_vertex_constraint)
    //   {
    //     ROS_INFO("Vertex Constraint: Agent: %s, Time: %f", constraint->second.agent_id.c_str(), constraint->second.time_step);
    //     for (double j : constraint->second.joint_pos_vertex->getJointPos())
    //     {
    //       ROS_INFO("%f, ", j);
    //     }
    //   }
    //   else
    //   {
    //     ROS_INFO("Edge Constraint: Agent: %s, Time: %f", constraint->second.agent_id.c_str(), constraint->second.time_step);
    //     auto joint_positions = *(constraint->second.joint_pos_edge->getVertexPositions());
    //     for (auto vec : joint_positions)
    //     {
    //       ROS_INFO("Vertex: ");
    //       for (double j : vec)
    //       {
    //         ROS_INFO("%f, ", j);
    //       }
    //     }
    //   }
    // }
    // if (vertex_constrained)
    // {
    //   ROS_INFO("Vertex Constrained!!!");
    //   // ROS_INFO("%s, %d, %f", constrained_vertex->second.agent_id.c_str(), constrained_vertex->second.is_vertex_constraint, constrained_vertex->second.time_step);

    //   ROS_INFO("Trying to travel to Vertex at time %f: ", current_time);
    //   for (double j : vertex->getJointPos())
    //   {
    //     ROS_INFO("%f, ", j);
    //   }

    //   if (constrained_vertex->second.is_vertex_constraint)
    //   {
    //     ROS_INFO("Vertex Constraint: Agent: %s, Time: %f", constrained_vertex->second.agent_id.c_str(), constrained_vertex->second.time_step);
    //     for (double j : constrained_vertex->second.joint_pos_vertex->getJointPos())
    //     {
    //       ROS_INFO("%f, ", j);
    //     }
    //   }
    //   else
    //   {
    //     ROS_INFO("Edge Constraint: Agent: %s, Time: %f", constrained_vertex->second.agent_id.c_str(), constrained_vertex->second.time_step);
    //     auto joint_positions = *(constrained_vertex->second.joint_pos_edge->getVertexPositions());
    //     for (auto vec : joint_positions)
    //     {
    //       ROS_INFO("Vertex: ");
    //       for (double j : vec)
    //       {
    //         ROS_INFO("%f, ", j);
    //       }
    //     }
    //   }
    // }
  }
  else
  {
    bool vertex_constrained = constrained_vertex != constraints.first.end();
    if (vertex_constrained)
    {
      for (auto c : constrained_vertex->second)
      {
        vertex_constrained = (c.time_step >= (current_time) && c.time_step <= (current_time + 2.0*timestep_)) ||
                            (c.time_step <= -1.0*(current_time) && c.time_step >= -1.0*(current_time + 2.0*timestep_));
        if (vertex_constrained)
          return true;
      }
    }
    // if (vertex_constrained)
    // {
    //   ROS_INFO("Vertex Constrained!!!");
    //   // ROS_INFO("%s, %d, %f", constrained_vertex->second.agent_id.c_str(), constrained_vertex->second.is_vertex_constraint, constrained_vertex->second.time_step);

    //   ROS_INFO("Trying to travel to Vertex at time %f: ", current_time);
    //   for (double j : vertex->getJointPos())
    //   {
    //     ROS_INFO("%f, ", j);
    //   }

    //   if (constrained_vertex->second.is_vertex_constraint)
    //   {
    //     ROS_INFO("Vertex Constraint: Agent: %s, Time: %f", constrained_vertex->second.agent_id.c_str(), constrained_vertex->second.time_step);
    //     for (double j : constrained_vertex->second.joint_pos_vertex->getJointPos())
    //     {
    //       ROS_INFO("%f, ", j);
    //     }
    //   }
    //   else
    //   {
    //     // ROS_INFO("Edge Constraint: Agent: %s, Time: %f", constrained_vertex->second.agent_id.c_str(), constrained_vertex->second.time_step);
    //     auto joint_positions = *(constrained_vertex->second.joint_pos_edge->getVertexPositions());
    //     for (auto vec : joint_positions)
    //     {
    //       ROS_INFO("Vertex: ");
    //       for (double j : vec)
    //       {
    //         ROS_INFO("%f, ", j);
    //       }
    //     }
    //   }
    // }

  }
  return false;  
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
