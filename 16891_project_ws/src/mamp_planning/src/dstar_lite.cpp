#include "mamp_planning/dstar_lite.hpp"

DStarLite::DStarLite(double timestep)
{
    timestep_ = timestep;
    path_time_ = -1;
    key_modifier_ = 0;

}

// bool DStarLite::computePRMPath(std::shared_ptr<Vertex> start, std::shared_ptr<Vertex> goal,
//                         std::pair<std::unordered_map<std::shared_ptr<Vertex>, std::vector<Constraint>>, std::unordered_map<std::shared_ptr<Edge>, std::vector<Constraint>>> &constraints,
//                         double max_constraint_time = 0)
// {
//     return true;
// }

bool DStarLite::isValid(std::shared_ptr<Vertex> vertex, std::shared_ptr<Edge> edge)
{
  return edge->isValid() && vertex->isValid();;
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
        edge_constrained = ((c.time_step >= (current_time - timestep_/2.0) && c.time_step <= (current_time + edge->getTraversalTime() + timestep_/2.0)) ||
                          (c.time_step <= -1.0*(current_time - timestep_/2.0) && c.time_step >= -1.0*(current_time + edge->getTraversalTime() + timestep_/2.0)));
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
                            (c.time_step <= -1.0*(current_time + edge->getTraversalTime() - timestep_) && c.time_step >= -1.0*(current_time + edge->getTraversalTime() + timestep_));
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
        vertex_constrained = (c.time_step >= (current_time) && c.time_step <= (current_time + 2.0*timestep_)) ||
                            (c.time_step <= -1.0*(current_time) && c.time_step >= -1.0*(current_time + 2.0*timestep_));
        if (vertex_constrained)
          return true;
      }
    }

  }
  return false;  
}


std::tuple<double, double> DStarLite::calculateKey(std::shared_ptr<Vertex> vertex, double vertex_time)
{ 
    // double g_vertex = g_.find(std::make_tuple(vertex->getId(),std::get<2>(vertex.first))).second;
    // double g_vertex = g_.find(std::make_tuple(vertex->getId(),vertex_time)).second;
    double g_vertex = g_[std::make_tuple(vertex->getId(),vertex_time)];
    double rhs_vertex = rhs_[std::make_tuple(vertex->getId(),vertex_time)];
    // double rhs_vertex = rhs_.find(std::make_tuple(vertex->getId(),vertex_time));
    double h_vertex = h_[vertex->getId()];

    double k1 = std::min(g_vertex, rhs_vertex) + h_vertex + key_modifier_;
    double k2 = std::min(g_vertex, rhs_vertex);
    return std::make_tuple(k1, k2);
}

double DStarLite::computeHeuristics(std::shared_ptr<Vertex> start)
{    
  OpenList<std::tuple<double, unsigned int>, Vertex, hash_tuple::hash<std::tuple<double, unsigned int>>> open_list_h;

  std::unordered_set<std::shared_ptr<Vertex>> closed_list_h;
  std::unordered_set<std::shared_ptr<Edge>> closed_edge_list;
  double max_time = 0;
  open_list_h.insert(std::make_tuple(0, start->getId()), start);
  // ROS_INFO("goal id %d", goal->getId());
  h_.insert({start->getId(), 0});

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


void DStarLite::initialize()
{
    // initialize the open list
    open_list_.clear();
    // initialize the closed list
    closed_list_.clear();
    // initialize the g and rhs values for all vertices to infinity
    g_.clear();
    rhs_.clear();

    // set the rhs value of the goal to 0
    rhs_[std::make_tuple(goal_->getId(), 0)] = 0;
    // insert the goal into the open list
    open_list_.insert(std::make_tuple(calculateKey(goal_, 0), goal_->getId(), 0), goal_);

}

void DStarLite::updateVertex(std::shared_ptr<Vertex> u, double current_time)
{
    if (u->getId() != goal_->getId())
    {
        // calculate the minimum rhs value of all successors
        double min_rhs = std::numeric_limits<double>::infinity();
        for (auto &edge : u->getEdges())
        {

            std::shared_ptr<Vertex> v = edge.second->getOpposingVertex(u);
            double cost = edge.second->getTraversalTime();
            if (rhs_[std::make_tuple(v->getId(), current_time)] + cost < min_rhs)
            {
                min_rhs = rhs_[std::make_tuple(v->getId(), current_time)] + cost;
            }
            
        }
        rhs_[std::make_tuple(u->getId(), current_time)] = min_rhs;
    }

    // if the vertex is in the open list, remove it
    if (open_list_.contains(std::make_tuple(calculateKey(u, current_time), u->getId(), current_time)))
    {
        // remove the vertex from the open list
        ROS_INFO("Not sure how to remove this element!");
        open_list_.pop_element(u);
    }

    // if g and rhs are not equal, insert the vertex into the open list
    double g_u = g_[std::make_tuple(u->getId(), current_time)];
    double rhs_u = rhs_[std::make_tuple(u->getId(), current_time)];
    if (g_u != rhs_u)
    {
        open_list_.insert(std::make_tuple(calculateKey(u, current_time), u->getId(), current_time + timestep_), u);
        // parent_map_.erase(std::make_tuple(u->getId(), current_time + timestep_));
        parent_map_.erase(std::make_tuple(u, current_time + timestep_));
        // parent_map_.insert({std::make_tuple(u, current_time + timestep_), std::make_tuple(u, current_time)});
        parent_map_.insert({std::make_tuple(u, current_time + timestep_), std::make_tuple(u, current_time)});
    }

}

void DStarLite::computePRMPath(std::shared_ptr<Vertex> start, std::shared_ptr<Vertex> goal,
                        std::pair<std::unordered_map<std::shared_ptr<Vertex>, std::vector<Constraint>>, std::unordered_map<std::shared_ptr<Edge>, std::vector<Constraint>>> &constraints,
                        double max_constraint_time)
{
  double max_time = computeHeuristics(start) + max_constraint_time;
  while (open_list_.size() > 0)
  {
    // auto u = open_list_.top();
    // get the key of the vertex at the top of the open list
    auto u_top = open_list_.top();
    // get the key of u_top from the open list
    auto k_old = std::get<0>(u_top.first);
        // while tjere is a vertex in the open list with a key less than the start vertex or the rhs value of the start vertex is not equal to the g value of the start vertex
        double current_time = std::get<2>(u_top.first);
        double rhs_start = rhs_[std::make_tuple(start_->getId(), current_time)];
        double g_start = g_[std::make_tuple(start_->getId(), current_time)];
        //TODO: how do I get the time of the start vertex?
        auto start_key = calculateKey(start, current_time);
        while(k_old < start_key || rhs_start != g_start)
        {
            auto k_new = calculateKey(u_top.second, std::get<2>(u_top.first));
            auto g_old = g_[std::make_tuple(u_top.second->getId(), std::get<2>(u_top.first))];
            auto u = open_list_.pop();
            double g_u = g_[std::make_tuple(u.second->getId(), std::get<2>(u.first))];
            double rhs_u = rhs_[std::make_tuple(u.second->getId(), std::get<2>(u.first))];
            if (k_old < k_new)
                // erase vertex with old key
                // open_list_.erase(std::make_tuple(k_old, u.second->getId(), current_time));
                // insert the vertex with the new key into the open list
                open_list_.insert(std::make_tuple(k_new, u.second->getId(), current_time), u.second);
            
            else if (g_u > rhs_u)
            {
                // update the g value of the vertex
                g_u = rhs_u;
                // insert this into g_
                g_.insert({std::make_tuple(u.second->getId(), std::get<2>(u.first) + timestep_), g_u});
                // for each predecessor of u which are all the nodes at the previous time step
                for (auto &edge : u.second->getEdges())
                {
                    if (isValid(edge.first, edge.second) && !isConstrained(edge.first, edge.second, current_time - edge.second->getTraversalTime(), constraints) && current_time - edge.second->getTraversalTime() <= max_time)
                    {
                    //     // auto u2 = Edge::getOppositeVertex(u);
                    //     // get all the vertices at the previous time step
                    //     double travel_time = edge.second->getTraversalTime();
                    //     // find all the vertices at the timestep current_time - travel_time
                        auto u2 = edge.second->getOpposingVertex(u.second);
                    //     auto u2_id = u2->getId();
                    //     auto u2_time = current_time - travel_time;
                   
                    // // auto u2 = Edge::getOppositeVertex(u);
                    // // get all the vertices at the previous time step
                    // double travel_time = edge.second->getTraversalTime();
                    // // find all the vertices at the timestep current_time - travel_time
                    // auto u2 = edge.second->getOpposingVertex(u.second);
                    // auto u2_id = u2->getId();
                    // auto u2_time = current_time - travel_time;
                    // // TODO: find the vertices with the same time 
                    // // update the vertex
                    updateVertex(u2, current_time - edge.second->getTraversalTime() + timestep_);
                    }


                    // update the vertex
                    // updateVertex(u2, current_time);
                
                }
            }
            else
            {
                auto min_rhs = std::numeric_limits<double>::infinity();
                g_old = g_u;
                g_.insert({std::make_tuple(u.second->getId(),std::get<2>(u.first) + timestep_), std::numeric_limits<double>::infinity()});
                // for each predecessor of u and u
                for (auto &edge : u.second->getEdges())
                {
                    auto g_neighbor = g_[std::make_tuple(edge.first->getId(), std::get<2>(u.first) + timestep_ + edge.second->getTraversalTime())];
                    if (isValid(edge.first, edge.second) && !isConstrained(edge.first, edge.second, current_time + edge.second->getTraversalTime(), constraints) && current_time + edge.second->getTraversalTime() <= max_time)
                    {
                        // update the vertex
                        if (rhs_u == edge.second->getTraversalTime() + g_neighbor)
                        {
                            // if vertex is not the goal vertex
                            if (u.second != goal)
                            {
                                // update the rhs value of the vertex
                                if (g_neighbor + edge.second->getTraversalTime() < min_rhs)
                                {
                                    min_rhs = g_neighbor + edge.second->getTraversalTime();

                                    rhs_.insert({std::make_tuple(u.second->getId(), std::get<2>(u.first) + timestep_), min_rhs});
                                }
                        
                            }
                        }
                        updateVertex(u.second, current_time + edge.second->getTraversalTime() + timestep_);
                    }
                }
            }
  }

  path_time_ = current_time;
}
}


std::vector<std::shared_ptr<Vertex>> DStarLite::getPRMPath(std::shared_ptr<Vertex> start, std::shared_ptr<Vertex> goal, std::pair<std::unordered_map<std::shared_ptr<Vertex>, std::vector<Constraint>>, std::unordered_map<std::shared_ptr<Edge>, std::vector<Constraint>>> &constraints, double max_time)
{
    std::vector<std::shared_ptr<Vertex>> prm_path;
    auto edge_cost = 0;
    // the current node is the start node
    std::shared_ptr<Vertex> last = start;
    std::shared_ptr<Vertex> v = goal;
    std::tuple<std::shared_ptr<Vertex>, double> v_time = std::make_tuple(goal, path_time_);
    prm_path.push_back(v);
    double time = path_time_;
    double max_constraint_time = 0; //TODO: what should this be?
    // initialize 
    initialize();

    // compute the path
    computePRMPath(start, goal, constraints, max_constraint_time);
    // while the start vertex is not the goal vertex
    while(start->getId()!=goal->getId())
    {
        // if the rhs value of the start vertex is infinity, there is no path
        // INFO: how do I get the time of the start vertex?
        // if the rhs value of the start vertex is infinity, there is no path
        if(rhs_[std::make_tuple(start->getId(), path_time_)] == std::numeric_limits<double>::infinity())
        {
            std::cout << "No path found" << std::endl;
            return prm_path;
        }

        // s_star = argmin{c(s, s') + g(s')} for all successors s' of s

        // for all successors of the start vertex


        // move to the start vertex
        //INFO: Not sure if this is how it needs to be done?
        
        // Ideally, this should backtrack from the goal vertex to the start vertex in order of minimum cost?
        
        while (v!=start || time!=0)
        {
            v_time = parent_map_.find(v_time)->second;
            v = std::get<0>(v_time);
            time = std::get<1>(v_time);
           
            prm_path.push_back(v);
        }

        // for all directed edges with changed edge costs 
        //TODO: Have a function to get this 
        auto changed_edges = v->getChangedEdges();
        if (changed_edges.size() > 0)
        {
            key_modifier_ += h_[start->getId()];
            last = start;
            for (auto changed_edge : changed_edges)
        {
            auto c_old = changed_edge.second->getTraversalTime();
            // update the edge cost
            auto c_new = c_old + key_modifier_;
            changed_edge.second->setTraversalTime(c_old + key_modifier_);
            double min_rhs = std::numeric_limits<double>::infinity();
            auto u2 = changed_edge.second->getOpposingVertex(changed_edge.first);
            auto u2_id = u2->getId();
            double g_u2 = g_[std::make_tuple(u2_id, std::get<1>(v_time) + c_old)];
            double rhs_u2 = rhs_[std::make_tuple(u2_id, std::get<1>(v_time) + c_old)];


            if (c_old > c_new)
            {
                if (changed_edge.first->getId() != goal->getId())
                {
                    // update rhs value of the vertex
                    if (min_rhs > g_u2 + c_new)
                    {
                        min_rhs = g_u2 + c_new;
                        rhs_.insert({std::make_tuple(u2_id, std::get<1>(v_time) + c_old), min_rhs});
                    }

                }
            }
                if (rhs_u2 == c_old + g_u2)
                {
                    if (changed_edge.first->getId() != goal->getId())
                    {
                        for (auto &edge : u2->getEdges())
                            {
                                auto edge_cost = edge.second->getTraversalTime();
                                auto g_neighbor = g_[std::make_tuple(edge.first->getId(), std::get<1>(v_time) + c_old)];

                                
                        if (isValid(edge.first, edge.second) && !isConstrained(edge.first, edge.second, std::get<1>(v_time) + edge.second->getTraversalTime(), constraints) && std::get<1>(v_time) + edge.second->getTraversalTime() <= max_time)
                        {
                            // update the vertex
                            if (min_rhs > g_neighbor + c_new)
                            
                                    {
                                        min_rhs = g_neighbor + edge.second->getTraversalTime();

                                        rhs_.insert({std::make_tuple(u2_id, std::get<1>(v_time) + c_old), min_rhs});
                                    }
                            
                                }
                            }

                        }
                    }

                updateVertex(u2, std::get<1>(v_time) + c_old + timestep_);

                }

            

        }

        }


        // get the minimum cost edge from the start vertex
        double min_rhs = std::numeric_limits<double>::infinity();
        std::shared_ptr<Edge> min_edge;
        for(auto &edge : start->getEdges())
        {
            
                std::shared_ptr<Vertex> vert = edge.first;
                double cost = edge.second->getTraversalTime();
                double g_v = g_[std::make_tuple(vert->getId(), std::get<1>(v_time))];
                double total_cost = cost + g_v;
                if(total_cost < min_rhs)
                {
                    min_rhs = total_cost;
                    auto min_edge = edge.second;
                }
                
        

            // backtrack to the start vertex
            while(last != start)
            {
                prm_path.push_back(last);

            }
        }

    computePRMPath(start, goal, constraints, max_constraint_time);
        return prm_path;


        
    }








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








