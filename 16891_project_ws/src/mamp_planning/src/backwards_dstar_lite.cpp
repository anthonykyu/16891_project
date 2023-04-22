#include "mamp_planning/backwards_dstar_lite.hpp"

BackwardsDStarLite::BackwardsDStarLite(double timestep)
{
  timestep_ = timestep;
  path_time_ = 0;
}

double BackwardsDStarLite::getG(std::tuple<unsigned int, double> vertex)
{
  if (g_.find(vertex) != g_.end())
    return g_[vertex];
  else
    return std::numeric_limits<double>::infinity();
}

void BackwardsDStarLite::setG(std::tuple<unsigned int, double> key, double value)
{
  g_[key] = value;
}

double BackwardsDStarLite::getRHS(std::tuple<unsigned int, double> vertex)
{
  if (rhs_.find(vertex) != rhs_.end())
    return rhs_[vertex];
  else
    return std::numeric_limits<double>::infinity();
}

void BackwardsDStarLite::setRHS(std::tuple<unsigned int, double> key, double value)
{
  rhs_[key] = value;
}


std::tuple<double, double> BackwardsDStarLite::calculateKey(std::tuple<unsigned int, double> vertex)
{
  double k1 = std::min(getG(vertex), getRHS(Vertex)) + h_[std::get<0>(vertex)];
  double k2 = std::min(getG(vertex), getRHS(Vertex));
  return std::make_tuple(k1, k2);
}

void BackwardsDStarLite::initialize(std::shared_ptr<Vertex> start, std::shared_ptr<Vertex> goal)
{
  // // initialize the open list
  // open_list_.clear();
  // // initialize the closed list
  // // closed_list_.clear();
  // // initialize the g and rhs values for all vertices to infinity
  // g_.clear();
  // rhs_.clear();
  // parent_map_.clear();
  // h_.clear();

  // set the rhs value of the start to 0
  rhs_[std::make_tuple(start->getId(), 0)] = 0;
  // calculate heuristics
  max_roadmap_traversal_time_ = computeHeuristics(goal);
  // insert the start into the open list
  open_list_.insert(std::make_tuple(h_[start->getId()], 0, start->getId(), 0), start);
  key_map_.insert({std::make_tuple(start->getId(), 0), std::make_tuple(h_[start->getId()], 0)});
}

void BackwardsDStarLite::updateVertex(std::tuple<unsigned int, double> vertex, std::shared_ptr<Vertex> vertex_ptr)
{
  if (getG(vertex) != getRHS(vertex))
  {
    // insert the vertex into the open list
    auto key = calculateKey(vertex);
    open_list_.insert(std::tuple_cat(key, vertex), vertex_ptr);
    key_map_[vertex] = key;
  }
  else if (getG(vertex) == getRHS(vertex) && key_map_.find(vertex) != key_map_.end())
  {
    // remove the vertex from the open list
    open_list_.removeByKey(std::tuple_cat(key_map_[vertex], vertex));
  }
}


bool BackwardsDStarLite::computeWaypointPaths(std::vector<std::shared_ptr<Vertex>> waypoints,
                        std::pair<std::unordered_map<std::shared_ptr<Vertex>, std::vector<Constraint>>, std::unordered_map<std::shared_ptr<Edge>, std::vector<Constraint>>> &constraints,
                        double max_constraint_time)
{
  bool success = true;
  path_time_ = 0;
  for (int i = 0; i < waypoints.size()-1; ++i)
  {
    if (i == waypoints.size()-1) 
      success = computePRMPath(waypoints[i], waypoints[i+1], constraints, max_constraint_time, path_time_, true);
    else
      success = computePRMPath(waypoints[i], waypoints[i+1], constraints, max_constraint_time, path_time_, false);
  }
  return success;
}

bool BackwardsDStarLite::computePRMPath(std::shared_ptr<Vertex> start, std::shared_ptr<Vertex> goal, 
std::pair<std::unordered_map<std::shared_ptr<Vertex>, std::vector<Constraint>>, std::unordered_map<std::shared_ptr<Edge>, std::vector<Constraint>>> &constraints,
double max_constraint_time, double start_time, bool is_last_waypoint)
{
  
}

std::vector<std::shared_ptr<Vertex>> BackwardsDStarLite::getPRMPath(std::shared_ptr<Vertex> start, std::shared_ptr<Vertex> goal)
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

double BackwardsDStarLite::computeHeuristics(std::shared_ptr<Vertex> goal)
{
  h_.clear();
  OpenList<std::tuple<double, unsigned int>, Vertex, hash_tuple::hash<std::tuple<double, unsigned int>>> open_list_h;
  std::unordered_set<std::shared_ptr<Vertex>> closed_list_h;
  std::unordered_set<std::shared_ptr<Edge>> closed_edge_list;
  double max_time = 0;
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
            isValid(neighbor.first, neighbor.second))  // Valid edge is not in the open list yet
        {
          if (closed_edge_list.find(neighbor.second) == closed_edge_list.end())
          {
            max_time += neighbor.second->getTraversalTime();
            closed_edge_list.insert(neighbor.second);
          }
          if (h_.find(neighbor.first->getId()) != h_.end()) // We have encountered this neighbor before
          {
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
            double new_h = h_.find(v->getId())->second + neighbor.second->getTraversalTime();
            h_.insert({neighbor.first->getId(), new_h});
            open_list_h.insert(std::make_tuple(new_h, neighbor.first->getId()), neighbor.first);
          }          
        }
      }
    }
  }
  return max_time;
}

bool BackwardsDStarLite::isConstrained(std::shared_ptr<Vertex> vertex, std::shared_ptr<Edge> edge, double current_time,
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

bool BackwardsDStarLite::isValid(std::shared_ptr<Vertex> vertex, std::shared_ptr<Edge> edge)
{
  return edge->isValid() && vertex->isValid();;
}