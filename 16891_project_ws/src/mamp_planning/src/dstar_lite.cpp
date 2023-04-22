#include "mamp_planning/dstar_lite.hpp"

DStarLite::DStarLite(std::shared_ptr<PRM> prm, double timestep)
{
  timestep_ = timestep;
  path_time_ = -1;
  key_modifier_ = 0;
  prm_ = prm;
}

// bool DStarLite::computePRMPath(std::shared_ptr<Vertex> start, std::shared_ptr<Vertex> goal,
//                         std::pair<std::unordered_map<std::shared_ptr<Vertex>, std::vector<Constraint>>, std::unordered_map<std::shared_ptr<Edge>, std::vector<Constraint>>> &constraints,
//                         double max_constraint_time = 0)
// {
//     return true;
// }

bool DStarLite::isValid(std::shared_ptr<Vertex> vertex, std::shared_ptr<Edge> edge)
{
  return edge->isValid() && vertex->isValid();
  ;
}

bool DStarLite::isConstrained(std::shared_ptr<Vertex> vertex, std::shared_ptr<Edge> edge, double current_time,
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
        edge_constrained = ((c.time_step >= (current_time - timestep_ / 2.0) && c.time_step <= (current_time + edge->getTraversalTime() + timestep_ / 2.0)) ||
                            (c.time_step <= -1.0 * (current_time - timestep_ / 2.0) && c.time_step >= -1.0 * (current_time + edge->getTraversalTime() + timestep_ / 2.0)));
        if (edge_constrained)
        {
          return true;
        }
      }
    }

    bool vertex_constrained = constrained_vertex != constraints.first.end();
    if (vertex_constrained)
    {
      for (auto c : constrained_vertex->second)
      {

        vertex_constrained = (c.time_step >= (current_time + edge->getTraversalTime() - timestep_) && c.time_step <= (current_time + edge->getTraversalTime() + timestep_)) ||
                             (c.time_step <= -1.0 * (current_time + edge->getTraversalTime() - timestep_) && c.time_step >= -1.0 * (current_time + edge->getTraversalTime() + timestep_));
        // ROS_INFO("Vertex Constrained: %d", vertex_constrained);
        if (vertex_constrained)
          return true;
      }
    }
  }
  else
  {
    bool vertex_constrained = constrained_vertex != constraints.first.end();
    if (vertex_constrained)
    {
      for (auto c : constrained_vertex->second)
      {
        vertex_constrained = (c.time_step >= (current_time) && c.time_step <= (current_time + 2.0 * timestep_)) ||
                             (c.time_step <= -1.0 * (current_time) && c.time_step >= -1.0 * (current_time + 2.0 * timestep_));
        if (vertex_constrained)
          return true;
      }
    }
  }
  return false;
}

std::tuple<double, double> DStarLite::calculateKey(std::shared_ptr<Vertex> vertex)
{
  // double g_vertex = g_.find(std::make_tuple(vertex->getId(),std::get<2>(vertex.first))).second;
  // double g_vertex = g_.find(std::make_tuple(vertex->getId(),vertex_time)).second;
  double g_vertex = g_[vertex->getId()];
  double rhs_vertex = rhs_[vertex->getId()];
  // double rhs_vertex = rhs_.find(std::make_tuple(vertex->getId(),vertex_time));
  double h_vertex = h_[vertex->getId()];

  double k1 = std::min(g_vertex, rhs_vertex) + h_vertex + key_modifier_;
  double k2 = std::min(g_vertex, rhs_vertex);
  return std::make_tuple(k1, k2);
}

bool DStarLite::compareKeys(std::tuple<double, double> key1, std::tuple<double, double> key2)
{
  if (std::get<0>(key1) < std::get<0>(key2))
  {
    return true;
  }
  else if (std::get<0>(key1) == std::get<0>(key2) && std::get<1>(key1) < std::get<1>(key2))
  {
    return true;
  }
  return false;
}

double DStarLite::computeHeuristics(std::shared_ptr<Vertex> start)
{
  OpenList<std::tuple<double, unsigned int>, std::tuple<unsigned int>, Vertex, hash_tuple::hash<std::tuple<unsigned int>>> open_list_h;

  std::unordered_set<std::shared_ptr<Vertex>> closed_list_h;
  std::unordered_set<std::shared_ptr<Edge>> closed_edge_list;
  double max_time = 0;
  open_list_h.insert(std::make_tuple(0, start->getId()), std::make_tuple(start->getId()), start);
  // ROS_INFO("goal id %d", goal->getId());
  h_.insert({start->getId(), 0});

  while (open_list_h.size() != 0)
  {
    std::shared_ptr<Vertex> v = std::get<2>(open_list_h.pop());
    if (closed_list_h.insert(v).second)
    {
      // ROS_INFO("Inserted vertex into closed list, getting neighbors now" );
      // ROS_INFO("Number of neighbors: %ld", v->getEdges().size());
      for (auto neighbor : v->getEdges())
      {
        // ROS_INFO("closed list_h size %d", closed_list_h.size());
        // ROS_INFO("isValid value %d", isValid(neighbor.first, neighbor.second));
        if (closed_list_h.find(neighbor.first) == closed_list_h.end() &&
            isValid(neighbor.first, neighbor.second)) // Valid edge is not in the open list yet
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
            double new_h = h_.find(v->getId())->second + getEdgeCost(neighbor.second);
            if (old_h > new_h)
            {
              h_.erase(neighbor.first->getId());
              h_.insert({neighbor.first->getId(), new_h});
              open_list_h.insert(std::make_tuple(new_h, neighbor.first->getId()), std::make_tuple(neighbor.first->getId()), neighbor.first);
            }
          }
          else // Seeing this neighbor for the first time
          {
            // ROS_INFO("Increasing open list");
            double new_h = h_.find(v->getId())->second + getEdgeCost(neighbor.second);
            h_.insert({neighbor.first->getId(), new_h});
            open_list_h.insert(std::make_tuple(new_h, neighbor.first->getId()), std::make_tuple(neighbor.first->getId()), neighbor.first);
          }
          // ROS_INFO("open list_h size %d", open_list_h.size());
        }
      }
    }
  }
  return max_time;
}

void DStarLite::initialize()
{
  // initialize the open list
  open_list_.clear();
  // initialize the closed list
  // closed_list_.clear();
  // initialize the g and rhs values for all vertices to infinity
  g_.clear();
  rhs_.clear();

  // set the rhs and g values of all the vertices in the PRM to infinity

  for (std::shared_ptr<Vertex> v : prm_->PRMgraph_)
  {
    g_.insert({v->getId(), std::numeric_limits<double>::infinity()});
    rhs_.insert({v->getId(), std::numeric_limits<double>::infinity()});
  }

  // set the rhs value of the goal to 0
  rhs_[goal_->getId()] = 0;
  // insert the goal into the open list
  open_list_.insert(std::make_tuple(std::get<0>(calculateKey(goal_)), std::get<1>(calculateKey(goal_)), goal_->getId()), std::make_tuple(goal_->getId()), goal_);
}

void DStarLite::updateVertex(std::shared_ptr<Vertex> u)
{

  if (g_[u->getId()] != rhs_[u->getId()])
  {
    // insert the vertex into the open list
    open_list_.insert(std::make_tuple(std::get<0>(calculateKey(u)), std::get<1>(calculateKey(u)), u->getId()), std::make_tuple(u->getId()), u);
  }
  else
  {
    // remove the vertex from the open list
    open_list_.remove(u->getId());
  }

  //   if (u->getId() != goal_->getId())
  //   {
  //     // calculate the minimum rhs value of all successors
  //     double min_rhs = std::numeric_limits<double>::infinity();
  //     for (auto &edge : u->getEdges())
  //     {

  //       std::shared_ptr<Vertex> v = edge.second->getOpposingVertex(u);
  //       double cost = edge.second->getTraversalTime();
  //       if (rhs_[v->getId()] + cost < min_rhs)
  //       {
  //         min_rhs = rhs_[v->getId()] + cost;
  //       }
  //     }
  //     rhs_[u->getId()] = min_rhs;
  //   }

  //   // if the vertex is in the open list, remove it
  //   if (open_list_.contains(std::make_tuple(calculateKey(u), u->getId())))
  //   {
  //     // remove the vertex from the open list
  //     ROS_INFO("Not sure how to remove this element!");
  //     open_list_.pop_element(u);
  //   }

  //   // if g and rhs are not equal, insert the vertex into the open list
  //   double g_u = g_[u->getId()];
  //   double rhs_u = rhs_[u->getId()];
  //   if (g_u != rhs_u)
  //   {
  //     open_list_.insert(std::make_tuple(calculateKey(u), u->getId()), u);

  //     // parent_map_.erase(std::make_tuple(u->getId(), current_time + timestep_));
  //     // parent_map_.erase(std::make_tuple(u, current_time + timestep_));
  //     // parent_map_.insert({std::make_tuple(u, current_time + timestep_), std::make_tuple(u, current_time)});
  //     // parent_map_.insert({std::make_tuple(u, current_time + timestep_), std::make_tuple(u, current_time)});
  //   }
}

double DStarLite::getEdgeCost(std::shared_ptr<Edge> edge)
{
  if (changed_edges_.find(edge) == changed_edges_.end())
  {
    return edge->getTraversalTime();
  }
  else
  {
    return changed_edges_[edge];
  }
}

// ComputeShortestPath function from the D*Lite algorithm
void DStarLite::computePRMPath(std::shared_ptr<Vertex> start, std::shared_ptr<Vertex> goal,
                               std::pair<std::unordered_map<std::shared_ptr<Vertex>, std::vector<Constraint>>, std::unordered_map<std::shared_ptr<Edge>, std::vector<Constraint>>> &constraints)
{
  // double max_time = computeHeuristics(start) + max_constraint_time;
  computeHeuristics(start);
  
  // auto u = open_list_.top();
  // get the key of the vertex at the top of the open list
  auto u_top = open_list_.top();
  // get the key of u_top from the open list
  std::tuple<double, double> k_old = std::make_tuple(std::get<0>(std::get<0>(u_top)), std::get<1>(std::get<0>(u_top)));
  // while tjere is a vertex in the open list with a key less than the start vertex or the rhs value of the start vertex is not equal to the g value of the start vertex
  // double current_time = std::get<2>(u_top.first);
  double rhs_start = rhs_[start_->getId()];
  double g_start = g_[start_->getId()];
  std::tuple<double, double> start_key = calculateKey(start);
  while (compareKeys(k_old, start_key) || rhs_start > g_start)
  {
    std::tuple<double, double> k_new = calculateKey(std::get<2>(u_top));
    double g_old = g_[std::get<2>(u_top)->getId()];
    std::shared_ptr<Vertex> u = std::get<2>(open_list_.pop());
    // double g_u = g_[u->getId()];
    double rhs_u = rhs_[u->getId()];
    if (compareKeys(k_old, k_new)) // k_old < k_new
      // erase vertex with old key
      // open_list_.erase(std::make_tuple(k_old, u.second->getId(), current_time));
      // insert the vertex with the new key into the open list
      open_list_.insert(std::make_tuple(std::get<0>(k_new), std::get<1>(k_new), u->getId()), std::make_tuple(u->getId()), u);

    else if (g_[u->getId()] > rhs_[u->getId()])
    {
      // update the g value of the vertex
      g_[u->getId()] = rhs_[u->getId()];
      // insert this into g_
      // erase the vertex from the open list
      open_list_.remove(u->getId());
      // for each predecessor of u which are all the nodes at the previous time step
      std::shared_ptr<Vertex> pred = u->getParent();
      if (u != goal)
      {
        // double cost;
        // if (pred->getEdges().find(u)->second)
        double cost = getEdgeCost(pred->getEdges().find(u)->second);
        
        // double cost = pred->getEdges().find(u)->second->getTraversalTime();
        
        rhs_[u->getId()] = min(rhs_[u->getId()], cost + g_[u->getId()]);
      }
      updateVertex(pred);
    }

    else
    {
      g_old = g_[u->getId()];
      g_[u->getId()] = std::numeric_limits<double>::infinity();
      // for each predecessor of u and u
      std::shared_ptr<Vertex> pred = u->getParent();

      std::vector<std::shared_ptr<Vertex>> temp_list{u, pred};
      double cost;
      for (std::shared_ptr<Vertex> s : temp_list)
      {
        if (s->getId() == u->getId())
          cost = 0;
        else
          cost = getEdgeCost(s->getEdges().find(u)->second);
          // cost = s->getEdges().find(u)->second->getTraversalTime();

        if (rhs_[s->getId()] == cost + g_old)
        {

          if (s->getId() != goal->getId())
          {
            double min_rhs = std::numeric_limits<double>::infinity();
            for (auto &succ : s->getEdges())
            {
              auto successor_v = succ.second->getOpposingVertex(s);
              if (successor_v == s)
                continue;
              if (g_[successor_v->getId()] + getEdgeCost(succ.second) < min_rhs)
              {
                min_rhs = g_[successor_v->getId()] + getEdgeCost(succ.second);
              }
            }
            rhs_[s->getId()] = min_rhs;
          }
        }
        // update the successor vertex
        updateVertex(s);
      }
    }
  }
}

double DStarLite::getTraversalTimeBetweenVertices(std::vector<std::shared_ptr<Vertex>> vertices, int idx_1, int idx_2)
{

  double time_sum = 0;
  
  for (int i=idx_1+1; i<idx_2+1; ++i)
  {
    std::shared_ptr<Edge> next_edge = vertices[i-1]->getEdges().find(vertices[i])->second;
    time_sum+=next_edge->getTraversalTime();
  }
  
  return time_sum;
}


// Main Function from the D* Lite Algorithm
std::vector<std::shared_ptr<Vertex>> DStarLite::getPRMPath(std::vector<std::shared_ptr<Vertex>> Astarpath, std::shared_ptr<Vertex> start, std::shared_ptr<Vertex> goal,
                                                           std::pair<std::unordered_map<std::shared_ptr<Vertex>, std::vector<Constraint>>, std::unordered_map<std::shared_ptr<Edge>, std::vector<Constraint>>> &constraints,
                                                           double max_time)
{
  std::vector<std::shared_ptr<Vertex>> prm_path;
  auto edge_cost = 0;
  // the current node is the start node
  // std::shared_ptr<Vertex> v = start;
  // std::shared_ptr<Vertex> v = start;

  // std::tuple<std::shared_ptr<Vertex>, double> v_time = std::make_tuple(goal, path_time_);
  double current_time = 0;
  // double time = path_time_;
  double max_constraint_time = max_time; // TODO: what should this be?
  // TODO: Call initialize in main cbs-mp
  // while the start vertex is not the goal vertex

  int last_idx = 0;
  int v_idx = 0;
  std::shared_ptr<Vertex> v = Astarpath[v_idx]; // start at the beginning of the path
  prm_path.push_back(v);

  while (v->getId() != goal->getId() || current_time < max_time)
  {

    // """
    // In the while loop:
    // 1. find next edge you will traverse
    // 2. Check if that edge is constrained for the time you will be on it
    // 3. Find the the next vertex
    // 4. Check if the next vertex is constrainted at the time you will be there
    // if (not constrained):
    //   add the next vertex to the prm_path
    //   continue
    // else:
    //   Do the replanning from line 37 in algorithm

    // """

    // Find the next edge / vertex, and the times we get there
    std::shared_ptr<Vertex> next_vertex = Astarpath[v_idx+1];
    std::shared_ptr<Edge> next_edge = v->getEdges().find(next_vertex)->second;
    if (isConstrained(next_vertex, next_edge, current_time, constraints))
    {
      // key_modifier_ += v->getEdges().find(next_vertex)->second->getTraversalTime();
      key_modifier_ += getTraversalTimeBetweenVertices(Astarpath, last_idx, v_idx);
      // last_idx = v_idx+1;

      double c_old = getEdgeCost(next_edge);
      changed_edges_.insert({next_edge, std::numeric_limits<double>::infinity()}); // Add the edge into changedEdges and set a new cost for it
      
      if (rhs_[v->getId()] == c_old + g_[v->getId()])
      {
        if (v->getId() != goal->getId())
        {
          double min_rhs = std::numeric_limits<double>::infinity();
          for (auto &succ : v->getEdges())
          {
            auto successor_v = succ.first;
            if (g_[successor_v->getId()] + getEdgeCost(succ.second) < min_rhs)
            {
              min_rhs = g_[successor_v->getId()] + getEdgeCost(succ.second);
            }
          }
          rhs_[v->getId()] = min_rhs;
        }
      }
      updateVertex(v);
      computePRMPath(v, goal, constraints); 
      std::shared_ptr<Vertex> u = v;
      Astarpath.clear();
      Astarpath.push_back(v);
      while(u->getId() != goal->getId())
      {
        double min_rhs = std::numeric_limits<double>::infinity();
        std::shared_ptr<Vertex> best_successor;
        for (auto succ : u->getEdges())
        {
          auto successor_u = succ.first;
          if (g_[successor_u->getId()] + getEdgeCost(succ.second) < min_rhs)
          {
            min_rhs = g_[successor_u->getId()] + getEdgeCost(succ.second);
            best_successor = successor_u;
          }
        }
        u = best_successor;
        Astarpath.push_back(u);
        // prm_path.push_back(v);
      }
      
      // last_idx = 0;
      v_idx = 0;
    }
    else
    {
      v_idx += 1;
      prm_path.push_back(next_vertex);
      current_time += v->getEdges().find(next_vertex)->second->getTraversalTime();
      v = next_vertex;
    }
  }
  return prm_path;
}









  //   // if the rhs value of the start vertex is infinity, there is no path
  //   if (rhs_[start->getId()] == std::numeric_limits<double>::infinity())
  //   {
  //     std::cout << "No path found" << std::endl;
  //     return;
  //   }

  //   // get new start vertex

  //   // TODO: Have a function to get this
  //   auto changed_edges = v->getChangedEdges();
  //   if (changed_edges.size() > 0)
  //   {
  //     key_modifier_ += h_[start->getId()];
  //     last = start;
  //     for (auto changed_edge : changed_edges)
  //     {
  //       auto c_old = changed_edge.second->getTraversalTime();
  //       // update the edge cost
  //       auto c_new = c_old + key_modifier_;
  //       changed_edge.second->setTraversalTime(c_old + key_modifier_);
  //       double min_rhs = std::numeric_limits<double>::infinity();
  //       auto u2 = changed_edge.second->getOpposingVertex(changed_edge.first);
  //       auto u2_id = u2->getId();
  //       double g_u2 = g_[u2_id];
  //       double rhs_u2 = rhs_[u2_id];

  //       if (c_old > c_new)
  //       {
  //         if (changed_edge.first->getId() != goal->getId())
  //         {
  //           // update rhs value of the vertex
  //           if (g_u2 + c_new < rhs_u2)
  //           {
  //             rhs_u2 = g_u2 + c_new;
  //             rhs_.insert({u2_id, rhs_u2});
  //           }
  //         }
  //       }
  //       if (rhs_u2 == c_old + g_u2)
  //       {
  //         if (changed_edge.first->getId() != goal->getId())
  //         {
  //           for (auto &edge : u2->getEdges())
  //           {
  //             auto edge_cost = edge.second->getTraversalTime();
  //             double g_neighbor = g_[edge.first->getId()];

  //             // if (isValid(edge.first, edge.second) && !isConstrained(edge.first, edge.second, std::get<1>(v_time) + edge.second->getTraversalTime(), constraints) && std::get<1>(v_time) + edge.second->getTraversalTime() <= max_time)
  //             // {
  //             // update the vertex
  //             if (g_neighbor + edge_cost < rhs_u2)
  //             {
  //               rhs_u2 = g_neighbor + edge_cost;
  //               rhs_.insert({u2_id, rhs_u2});
  //             }
  //           }
  //         }
  //       }

  //       updateVertex(u2);
  //     }
  //   }
  // }

  // // // get the minimum cost edge from the start vertex
  // // double min_rhs = std::numeric_limits<double>::infinity();
  // // std::shared_ptr<Edge> min_edge;
  // // for (auto &edge : start->getEdges())
  // // {

  // //   std::shared_ptr<Vertex> vert = edge.first;
  // //   double cost = edge.second->getTraversalTime();
  // //   double g_v = g_[std::make_tuple(vert->getId(), std::get<1>(v_time))];
  // //   double total_cost = cost + g_v;
  // //   if (total_cost < min_rhs)
  // //   {
  // //     min_rhs = total_cost;
  // //     auto min_edge = edge.second;
  // //   }

  // //   // backtrack to the start vertex
  // //   while (last != start)
  // //   {
  // //     prm_path.push_back(last);
  // //   }
  // // }

  // computePRMPath(start, goal, constraints, max_constraint_time);
  // return prm_path;
// }

// // double DStarLite::setRHS(std::shared_ptr<Vertex> vertex, double current_time,
// //                   std::pair<std::unordered_map<std::shared_ptr<Vertex>, std::vector<Constraint>>, std::unordered_map<std::shared_ptr<Edge>, std::vector<Constraint>>> &constraints)
// // {
// //     return 0;
// // }

// // double DStarLite::getRHS(std::shared_ptr<Vertex> vertex, double current_time,
// //                   std::pair<std::unordered_map<std::shared_ptr<Vertex>, std::vector<Constraint>>, std::unordered_map<std::shared_ptr<Edge>, std::vector<Constraint>>> &constraints)
// // {
// //     return 0;
// // }
