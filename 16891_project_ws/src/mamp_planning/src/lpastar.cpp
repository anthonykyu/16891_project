#include "mamp_planning/lpastar.hpp"

LPAStar::LPAStar(double timestep)
{
  timestep_ = timestep;
  path_time_ = 0;
  max_roadmap_traversal_time_ = 0;
}

LPAStar::LPAStar(std::shared_ptr<LPAStar> &d)
{
  // ROS_INFO("Check1");
  open_list_ = d->getOpenList();
  // ROS_INFO("Check2");
  g_ = d->getGMap();
  rhs_ = d->getRHSMap();
  h_ = d->getH();
  // ROS_INFO("Check3");
  timestep_ = d->getTimestep();
  max_roadmap_traversal_time_ = d->getMaxRoadmapTraversalTime();
  path_time_ = d->getPathTime();
  // ROS_INFO("Check4");
  expanded_goal_times_ = d->getExpandedGoalTimes();
  // ROS_INFO("Check5");
}

OpenList<std::tuple<unsigned int, unsigned int, unsigned int, unsigned int>, std::tuple<unsigned int, unsigned int>, Vertex, hash_tuple::hash<std::tuple<unsigned int, unsigned int>>> &LPAStar::getOpenList()
{
  return open_list_;
}

std::unordered_map<std::tuple<unsigned int, unsigned int>, unsigned int, hash_tuple::hash<std::tuple<unsigned int, unsigned int>>> &LPAStar::getGMap()
{
  return g_;
}

std::unordered_map<std::tuple<unsigned int, unsigned int>, unsigned int, hash_tuple::hash<std::tuple<unsigned int, unsigned int>>> &LPAStar::getRHSMap()
{
  return rhs_;
}

std::unordered_map<unsigned int, unsigned int> &LPAStar::getH()
{
  return h_;
}

unsigned int LPAStar::getMaxRoadmapTraversalTime()
{
  return max_roadmap_traversal_time_;
}

double LPAStar::getTimestep()
{
  return timestep_;
}

unsigned int LPAStar::getPathTime()
{
  return path_time_;
}

std::set<unsigned int> &LPAStar::getExpandedGoalTimes()
{
  return expanded_goal_times_;
}

unsigned int LPAStar::getG(std::tuple<unsigned int, unsigned int> vertex)
{
  if (g_.find(vertex) != g_.end())
    return g_[vertex];
  else
    return std::numeric_limits<unsigned int>::max();
}

void LPAStar::setG(std::tuple<unsigned int, unsigned int> key, unsigned int value)
{
  g_[key] = value;
}

unsigned int LPAStar::addToG(std::tuple<unsigned int, unsigned int> vertex, unsigned int in)
{
  unsigned int g = getG(vertex);
  return g == std::numeric_limits<unsigned int>::max() ? g : g + in;
}

unsigned int LPAStar::getRHS(std::tuple<unsigned int, unsigned int> vertex)
{
  if (rhs_.find(vertex) != rhs_.end())
    return rhs_[vertex];
  else
    return std::numeric_limits<unsigned int>::max();
}

void LPAStar::setRHS(std::tuple<unsigned int, unsigned int> key, unsigned int value)
{
  rhs_[key] = value;
}

std::tuple<unsigned int, unsigned int> LPAStar::calculateKey(std::tuple<unsigned int, unsigned int> vertex)
{
  unsigned int k1 = std::min(getG(vertex), getRHS(vertex)) + h_[std::get<0>(vertex)];
  unsigned int k2 = std::min(getG(vertex), getRHS(vertex));
  return std::make_tuple(k1, k2);
}

void LPAStar::initialize(std::shared_ptr<Vertex> start, std::shared_ptr<Vertex> goal)
{
  // // initialize the open list
  open_list_.clear();
  // // initialize the g and rhs values for all vertices to infinity
  g_.clear();
  rhs_.clear();
  h_.clear();
  expanded_goal_times_.clear();
  max_roadmap_traversal_time_ = 0;
  path_time_ = 0;

  // set the rhs value of the start to 0
  rhs_[std::make_tuple(start->getId(), 0)] = 0;
  // calculate heuristics
  max_roadmap_traversal_time_ = computeHeuristics(goal);
  // insert the start into the open list
  open_list_.insert(std::make_tuple(h_[start->getId()], 0, start->getId(), 0), std::make_tuple(start->getId(), 0), start);
  // key_map_.insert({std::make_tuple(start->getId(), 0), std::make_tuple(h_[start->getId()], 0)});
}

void LPAStar::updateVertex(std::tuple<unsigned int, unsigned int> vertex, std::shared_ptr<Vertex> vertex_ptr, std::shared_ptr<Vertex> goal)
{
  if (getG(vertex) != getRHS(vertex))
  {
    // insert the vertex into the open list
    // ROS_INFO("key: %f, %f, id: %d, time: %f", std::get<0>(calculateKey(vertex)), std::get<1>(calculateKey(vertex)), vertex_ptr->getId(), std::get<1>(vertex));
    open_list_.insert(std::tuple_cat(calculateKey(vertex), vertex), vertex, vertex_ptr);
    if (vertex_ptr->getId() == goal->getId())
      expanded_goal_times_.erase(std::get<1>(vertex));
    // key_map_[vertex] = key;
  }
  else if (getG(vertex) == getRHS(vertex))
  {
    // remove the vertex from the open list
    // ROS_INFO("REMOVED: %d, t: %f", std::get<0>(vertex), std::get<1>(vertex));
    open_list_.remove(vertex);
    if (vertex_ptr->getId() == goal->getId() && getG(vertex) != std::numeric_limits<unsigned int>::max())
      expanded_goal_times_.insert(std::get<1>(vertex));
  }
}

// bool LPAStar::computeWaypointPaths(std::vector<std::shared_ptr<Vertex>> waypoints,
//                                               std::pair<std::unordered_map<std::shared_ptr<Vertex>, std::vector<Constraint>>, std::unordered_map<std::shared_ptr<Edge>, std::vector<Constraint>>> &constraints,
//                                               double max_constraint_time)
// {
//   bool success = true;
//   path_time_ = 0;
//   for (int i = 0; i < waypoints.size() - 1; ++i)
//   {
//     if (i == waypoints.size() - 1)
//       success = computePRMPath(waypoints[i], waypoints[i + 1], constraints, max_constraint_time, path_time_, true);
//     else
//       success = computePRMPath(waypoints[i], waypoints[i + 1], constraints, max_constraint_time, path_time_, false);
//   }
//   return success;
// }

// void LPAStar::clearExpandedGoals(std::shared_ptr<Vertex> goal)
// {
//   for (double time: expanded_goal_times_)
//   {
//     std::tuple<unsigned int, double> goal_tuple = std::make_tuple(goal->getId(), round(time));
//     setG(goal_tuple, std::numeric_limits<double>::infinity());
//     setRHS(goal_tuple, std::numeric_limits<double>::infinity());
//     // updateVertex(goal_tuple, goal, goal);
//   }
//   expanded_goal_times_.clear();
// }

bool LPAStar::computePRMPath(std::shared_ptr<Vertex> start, std::shared_ptr<Vertex> goal,
                                        std::pair<std::unordered_map<std::shared_ptr<Vertex>, std::vector<Constraint>>, std::unordered_map<std::shared_ptr<Edge>, std::vector<Constraint>>> &constraints, 
                                        double max_constraint_time, std::vector<Constraint> new_constraints,
                                        double start_time, bool is_last_waypoint)
{
  // ROS_INFO("Start open_list_.size(): %ld", open_list_.size());
  // if (new_constraints.size() > 0)
  //   clearExpandedGoals(goal);
  num_expansions_ = 0;
  for (Constraint c : new_constraints)
  {
    unsigned int constraint_time = toDiscrete(c.time_step);
    if (c.is_vertex_constraint)
    {
      // ROS_INFO("current time vertex: %f", c.time_step);
      // vertex constraint
      // all predecessors
      if (c.joint_pos_vertex->getId() != start->getId() && constraint_time != 0)
      {
        setRHS(std::make_tuple(c.joint_pos_vertex->getId(), constraint_time), std::numeric_limits<unsigned int>::max());
        // ROS_INFO("Setting RHS to inf, ID: %d, Time: %f", c.joint_pos_vertex->getId(), round(constraint_time));
        updateVertex(std::make_tuple(c.joint_pos_vertex->getId(), constraint_time), c.joint_pos_vertex, goal);
      }

      // all successors
      // All PRM Neighbor edges
      for (auto succ : c.joint_pos_vertex->getEdges())
      {
        if (getRHS(std::make_tuple(succ.first->getId(), constraint_time + succ.second->getDivisions())) == addToG(std::make_tuple(c.joint_pos_vertex->getId(), constraint_time), succ.second->getDivisions()))
        {
          // Line 46, for all predecessors (backwards)
          unsigned int min_rhs = std::numeric_limits<unsigned int>::max();
          for (auto pred : succ.first->getEdges())
          {
            if (constraint_time + succ.second->getDivisions() >= pred.second->getDivisions() &&
              !isConstrained(succ.first, pred.second, constraint_time + succ.second->getDivisions() - pred.second->getDivisions(), constraints))
            {
              unsigned int newG = addToG(std::make_tuple(pred.first->getId(), constraint_time + succ.second->getDivisions() - pred.second->getDivisions()), pred.second->getDivisions());
              if (min_rhs > newG)
              {
                min_rhs = newG;
              }
            }
          }
          if (constraint_time + succ.second->getDivisions() >= 1 &&
            !isConstrained(succ.first, nullptr, constraint_time + succ.second->getDivisions() - 1, constraints))
          {
            unsigned int newG = addToG(std::make_tuple(succ.first->getId(), constraint_time + succ.second->getDivisions() - 1), 1);
            if (min_rhs > newG)
            {
              min_rhs = newG;
            }
          }
          // ROS_INFO("Setting Successor ID %d at time %f to RHS: %f", succ.first->getId(), constraint_time + succ.second->getDivisions(), min_rhs);
          setRHS(std::make_tuple(succ.first->getId(), constraint_time + succ.second->getDivisions()), min_rhs);
        }
        updateVertex(std::make_tuple(succ.first->getId(), constraint_time + succ.second->getDivisions()), succ.first, goal);
      }
      // Staying in same spot edge
      if (getRHS(std::make_tuple(c.joint_pos_vertex->getId(), constraint_time + 1)) == addToG(std::make_tuple(c.joint_pos_vertex->getId(), constraint_time), 1))
      {
        // Line 46, for all predecessors (backwards)
        unsigned int min_rhs = std::numeric_limits<unsigned int>::max();
        for (auto pred : c.joint_pos_vertex->getEdges())
        {
          if (constraint_time + 1 >= pred.second->getDivisions() &&
              !isConstrained(c.joint_pos_vertex, pred.second, constraint_time + 1 - pred.second->getDivisions(), constraints))
          {
            unsigned int newG = addToG(std::make_tuple(pred.first->getId(), constraint_time + 1 - pred.second->getDivisions()), pred.second->getDivisions());
            if (min_rhs > newG)
            {
              min_rhs = newG;
            }
          }
        }
        // ROS_INFO("Setting Successor ID %d at time %f to RHS: %f", c.joint_pos_vertex->getId(), round(constraint_time + 1), min_rhs);
        setRHS(std::make_tuple(c.joint_pos_vertex->getId(), constraint_time + 1), min_rhs);
      }
      updateVertex(std::make_tuple(c.joint_pos_vertex->getId(), constraint_time + 1), c.joint_pos_vertex, goal);
    }
    else
    {
      // Edge constraint
      unsigned int c_old = c.joint_pos_edge->getDivisions();
      unsigned int current_time = constraint_time >= c.joint_pos_edge->getDivisions() ? constraint_time - c.joint_pos_edge->getDivisions() : 0;
      for (; current_time <= constraint_time; ++current_time)
      {
        // ROS_INFO("current time edge: %f, timestep: %f, trav time: %f", current_time, c.time_step, c.joint_pos_edge->getDivisions());
        for (int i = 0; i < 2; ++i)
        {
          int j;
          int k;
          if (i == 0)
          {
            j = 0;
            k = 1;
          }
          else
          {
            j = 1;
            k = 0;
          }
          if (getRHS(std::make_tuple(c.joint_pos_edge->ordered_vertices_[j]->getId(), current_time + c.joint_pos_edge->getDivisions())) ==
              addToG(std::make_tuple(c.joint_pos_edge->ordered_vertices_[k]->getId(), current_time), c_old))
          {
            unsigned int min_rhs = std::numeric_limits<unsigned int>::max();
            for (auto pred : c.joint_pos_edge->ordered_vertices_[j]->getEdges())
            {
              if (current_time + c.joint_pos_edge->getDivisions() >= pred.second->getDivisions() &&
                  !isConstrained(c.joint_pos_edge->ordered_vertices_[j], pred.second, current_time + c.joint_pos_edge->getDivisions() - pred.second->getDivisions(), constraints))
              {
                unsigned int newG = addToG(std::make_tuple(pred.first->getId(), current_time + c.joint_pos_edge->getDivisions() - pred.second->getDivisions()), pred.second->getDivisions());
                if (min_rhs > newG)
                {
                  min_rhs = newG;
                }
              }
            }
            if (!isConstrained(c.joint_pos_edge->ordered_vertices_[j], nullptr, current_time + c.joint_pos_edge->getDivisions() - 1, constraints))
            {
              unsigned int newG = addToG(std::make_tuple(c.joint_pos_edge->ordered_vertices_[j]->getId(), current_time + c.joint_pos_edge->getDivisions() - 1), 1);
              if (min_rhs > newG)
              {
                min_rhs = newG;
              }
            }
            setRHS(std::make_tuple(c.joint_pos_edge->ordered_vertices_[j]->getId(), current_time + c.joint_pos_edge->getDivisions()), min_rhs);
          }
          updateVertex(std::make_tuple(c.joint_pos_edge->ordered_vertices_[j]->getId(), current_time + c.joint_pos_edge->getDivisions()), c.joint_pos_edge->ordered_vertices_[j], goal);
        }
      }
    }
  }
  return computeShortestPath(start, goal, constraints, max_constraint_time);
}

bool LPAStar::shortestPathSearchCondition(std::shared_ptr<Vertex> start, std::shared_ptr<Vertex> goal,
                         std::pair<std::unordered_map<std::shared_ptr<Vertex>, std::vector<Constraint>>, std::unordered_map<std::shared_ptr<Edge>, std::vector<Constraint>>> &constraints,
                         double max_constraint_time)
{
  if (open_list_.size() == 0)
  {
    ROS_ERROR("No Path!");
    return false;
  }

  for (unsigned int goal_time : expanded_goal_times_)
  {    
    auto top_key = std::get<0>(open_list_.top());
    std::tuple<unsigned int, unsigned int> goal_tuple = std::make_tuple(goal->getId(), goal_time);
    if (std::make_tuple(std::get<0>(top_key), std::get<1>(top_key)) >= calculateKey(goal_tuple) && getRHS(goal_tuple) <= getG(goal_tuple) && goal_time >= toDiscrete(max_constraint_time))
    {
      path_time_ = goal_time;
      return false;
    }
  }
  return true;
}

bool LPAStar::computeShortestPath(std::shared_ptr<Vertex> start, std::shared_ptr<Vertex> goal,
                         std::pair<std::unordered_map<std::shared_ptr<Vertex>, std::vector<Constraint>>, std::unordered_map<std::shared_ptr<Edge>, std::vector<Constraint>>> &constraints,
                         double max_constraint_time)
{
  unsigned int max_time = toDiscrete(max_constraint_time) + max_roadmap_traversal_time_;
  while(shortestPathSearchCondition(start, goal, constraints, max_constraint_time))
  {
    ++num_expansions_;
    // ROS_INFO("Updating");
    auto top = open_list_.top();
    // std::tuple<unsigned int, unsigned int> k_old = std::make_tuple(std::get<0>(std::get<0>(top)), std::get<1>(std::get<0>(top)));

    std::tuple<unsigned int, unsigned int> u_tuple = std::get<1>(top);
    unsigned int u_time = std::get<1>(u_tuple);
    if (u_time > max_time)
      continue;
    std::shared_ptr<Vertex> u = std::get<2>(top);
    // ROS_INFO("old key: %f, %f, id: %d, time: %f", std::get<0>(k_old), std::get<1>(k_old), u->getId(), std::get<1>(u_tuple));
    // std::tuple<unsigned int, unsigned int> k_new = calculateKey(u_tuple);
    // ROS_INFO("new key: %f, %f, id: %d, time: %f", std::get<0>(k_new), std::get<1>(k_new), u->getId(), std::get<1>(u_tuple));

    // if (k_old < k_new)
    // {
    //   // ROS_INFO("Update key");
    //   open_list_.insert(std::tuple_cat(k_new, u_tuple), u_tuple, u);
    //   updateVertex(u_tuple, u, goal);
    //   // double u_time = round(std::get<1>(u_tuple));
    //   // if (u->getId() == goal->getId())
    //   //   expanded_goal_times_.erase(u_time);
    // }
    // else 
    if (getG(u_tuple) > getRHS(u_tuple))
    {
      setG(u_tuple, getRHS(u_tuple));
      updateVertex(u_tuple, u, goal);
      for (auto succ : u->getEdges())
      {
        if (u_time + succ.second->getDivisions() <= max_time)
        {
          std::tuple<unsigned int, unsigned int> succ_tuple = std::make_tuple(succ.first->getId(), u_time + succ.second->getDivisions());
          if (!isConstrained(succ.first, succ.second, u_time, constraints) &&
          getRHS(succ_tuple) > addToG(u_tuple, succ.second->getDivisions()))
          {
            setRHS(succ_tuple, addToG(u_tuple, succ.second->getDivisions()));
            // std::tuple<std::shared_ptr<Vertex>, unsigned int> succ_tuple_v = std::make_tuple(succ.first, u_time + succ.second->getDivisions());
            // std::tuple<std::shared_ptr<Vertex>, unsigned int> u_tuple_v = std::make_tuple(u, u_time);
            // parent_map_.erase(succ_tuple_v);
            // parent_map_.insert({succ_tuple_v, u_tuple_v});
          }
          updateVertex(succ_tuple, succ.first, goal);
        }
      }
      if (u_time + 1 <= max_time)
      {
        std::tuple<unsigned int, unsigned int> succ_tuple = std::make_tuple(u->getId(), u_time + 1);
        if (!isConstrained(u, nullptr, u_time, constraints) &&
        getRHS(succ_tuple) > addToG(u_tuple, 1))
        {
          setRHS(succ_tuple, addToG(u_tuple, 1));
          // std::tuple<std::shared_ptr<Vertex>, unsigned int> succ_tuple_v = std::make_tuple(u, u_time + 1);
          // std::tuple<std::shared_ptr<Vertex>, unsigned int> u_tuple_v = std::make_tuple(u, u_time);
          // parent_map_.erase(succ_tuple_v);
          // parent_map_.insert({succ_tuple_v, u_tuple_v});
        }
        updateVertex(succ_tuple, u, goal);
      }
    }
    else
    {
      // ROS_INFO("Reset key");
      unsigned int g_old = getG(u_tuple);
      setG(u_tuple, std::numeric_limits<unsigned int>::max());
      // if (u->getId() == goal->getId())
      //   expanded_goal_times_.erase(u_time);
      for (auto succ : u->getEdges())
      {
        if (u_time + succ.second->getDivisions() <= max_time)
        {
          std::tuple<unsigned int, unsigned int> succ_tuple = std::make_tuple(succ.first->getId(), u_time + succ.second->getDivisions());
          unsigned int addedG = g_old == std::numeric_limits<unsigned int>::max() ? g_old : g_old + succ.second->getDivisions();
          if (getRHS(succ_tuple) == addedG)
          {
            // Line 46, for all predecessors (backwards)
            unsigned int min_rhs = std::numeric_limits<unsigned int>::max();
            for (auto pred : succ.first->getEdges())
            {
              std::tuple<unsigned int, unsigned int> pred_tuple = std::make_tuple(pred.first->getId(), u_time + succ.second->getDivisions() - pred.second->getDivisions());
              if (u_time + succ.second->getDivisions() >= pred.second->getDivisions() &&
                  !isConstrained(succ.first, pred.second, u_time + succ.second->getDivisions() - pred.second->getDivisions(), constraints) &&
                  min_rhs > addToG(pred_tuple, pred.second->getDivisions()))
              {
                min_rhs = addToG(pred_tuple, pred.second->getDivisions());
              }
            }
            std::tuple<unsigned int, unsigned int> pred_tuple = std::make_tuple(succ.first->getId(), u_time + succ.second->getDivisions() - 1);
            if (!isConstrained(succ.first, nullptr, u_time + succ.second->getDivisions() - 1, constraints) &&
                min_rhs > addToG(pred_tuple, 1))
            {
              min_rhs = addToG(pred_tuple, 1);
            }
            setRHS(succ_tuple, min_rhs);
          }
          updateVertex(succ_tuple, succ.first, goal);
        }
      }
      // Staying in same spot successor
      if (u_time + 1 <= max_time)
      {
        std::tuple<unsigned int, unsigned int> succ_tuple = std::make_tuple(u->getId(), u_time + 1);
        unsigned int addedG = g_old == std::numeric_limits<unsigned int>::max() ? g_old : g_old + 1;
        if (getRHS(succ_tuple) == addedG)
        {
          // Line 46, for all predecessors (backwards)
          unsigned int min_rhs = std::numeric_limits<unsigned int>::max();
          for (auto pred : u->getEdges())
          {
            std::tuple<unsigned int, unsigned int> pred_tuple = std::make_tuple(pred.first->getId(), u_time + 1 - pred.second->getDivisions());
            if (u_time + 1 >= pred.second->getDivisions() &&
                !isConstrained(u, pred.second, u_time + 1 - pred.second->getDivisions(), constraints) &&
                min_rhs > addToG(pred_tuple, pred.second->getDivisions()))
            {
              min_rhs = addToG(pred_tuple, pred.second->getDivisions());
            }
          }
          std::tuple<unsigned int, unsigned int> pred_tuple = std::make_tuple(u->getId(), u_time);
          if (!isConstrained(u, nullptr, u_time, constraints) &&
              min_rhs > addToG(pred_tuple, 1))
          {
            min_rhs = addToG(pred_tuple, 1);
          }
          setRHS(succ_tuple, min_rhs);
        }
        updateVertex(succ_tuple, u, goal);
      }
      
      // same spot, same time, u
      if (getRHS(u_tuple) == g_old && u->getId() != start->getId() && u_time != 0)
      {
        unsigned int min_rhs = std::numeric_limits<unsigned int>::max();
        for (auto pred : u->getEdges())
        {
          std::tuple<unsigned int, unsigned int> pred_tuple = std::make_tuple(pred.first->getId(), u_time - pred.second->getDivisions());
          if (u_time >= pred.second->getDivisions() &&
              !isConstrained(u, pred.second, u_time - pred.second->getDivisions(), constraints) &&
              min_rhs > addToG(pred_tuple, pred.second->getDivisions()))
          {
            min_rhs = addToG(pred_tuple, pred.second->getDivisions());
          }
        }
        std::tuple<unsigned int, unsigned int> pred_tuple = std::make_tuple(u->getId(), u_time - 1);
        if (u_time >= 1 &&
            !isConstrained(u, nullptr, u_time - 1, constraints) &&
            min_rhs > addToG(pred_tuple, 1))
        {
          min_rhs = addToG(pred_tuple, 1);
        }
        setRHS(u_tuple, min_rhs);
      }
      updateVertex(u_tuple, u, goal);
    }
  }
  // ROS_WARN("NUMBER OF EXPANSIONS: %d", num_expansions_);
  if (open_list_.size() == 0)
    return false;
  return true;
}

std::vector<std::shared_ptr<Vertex>> LPAStar::getPRMPath(std::shared_ptr<Vertex> start, std::shared_ptr<Vertex> goal,
  std::pair<std::unordered_map<std::shared_ptr<Vertex>, std::vector<Constraint>>, std::unordered_map<std::shared_ptr<Edge>, std::vector<Constraint>>> &constraints)
{
  std::vector<std::shared_ptr<Vertex>> prm_path;
  std::shared_ptr<Vertex> v = goal;
  // std::tuple<std::shared_ptr<Vertex>, unsigned int> v_time = std::make_tuple(goal, path_time_);
  prm_path.push_back(v);
  unsigned int time = path_time_;
  while (v->getId() != start->getId() || time != 0)
  {
    std::shared_ptr<Vertex> s = v;
    unsigned int pred_time = time;
    unsigned int f = std::numeric_limits<unsigned int>::max();
    for (auto pred : v->getEdges())
    {
      if (time >= pred.second->getDivisions() &&
          !isConstrained(v, pred.second, time - pred.second->getDivisions(), constraints) &&
          f > addToG(std::make_tuple(pred.first->getId(), time - pred.second->getDivisions()), pred.second->getDivisions()))
      {
        f = addToG(std::make_tuple(pred.first->getId(), time - pred.second->getDivisions()), pred.second->getDivisions());
        s = pred.first;
        pred_time = time - pred.second->getDivisions();
      }
    }
    if (time >= 1 &&
        !isConstrained(v, nullptr, time - 1, constraints) &&
        f > addToG(std::make_tuple(v->getId(), time - 1), 1))
    {
      f = addToG(std::make_tuple(v->getId(), time - 1), 1);
      s = v;
      pred_time = time - 1;
    }
    v = s;
    time = pred_time;
    prm_path.push_back(v);
  }
  // while (v != start || time != 0)
  // {
  //   ROS_INFO("CHeck1");
  //   v_time = parent_map_.find(v_time)->second;
  //   ROS_INFO("CHeck2");
  //   v = std::get<0>(v_time);
  //   time = std::get<1>(v_time);
  //   prm_path.push_back(v);
  // }
  std::reverse(prm_path.begin(), prm_path.end());
  return prm_path;
}

unsigned int LPAStar::getNumExpansions()
{
  return num_expansions_;
}

unsigned int LPAStar::computeHeuristics(std::shared_ptr<Vertex> goal)
{
  OpenList<std::tuple<unsigned int, unsigned int>, std::tuple<unsigned int>, Vertex, hash_tuple::hash<std::tuple<unsigned int>>> open_list_h;
  std::unordered_set<std::shared_ptr<Vertex>> closed_list_h;
  std::unordered_set<std::shared_ptr<Edge>> closed_edge_list;
  unsigned int max_time = 0;
  open_list_h.insert(std::make_tuple(0, goal->getId()), std::make_tuple(goal->getId()), goal);
  // ROS_INFO("goal id %d", goal->getId());
  h_.insert({goal->getId(), 0});

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
            max_time += neighbor.second->getDivisions();
            closed_edge_list.insert(neighbor.second);
          }
          // ROS_INFO("Neighbor cost %f", neighbor.second->getDivisions());
          if (h_.find(neighbor.first->getId()) != h_.end()) // We have encountered this neighbor before
          {
            // ROS_INFO("Keeping same size of open list");
            unsigned int old_h = h_.find(neighbor.first->getId())->second;
            unsigned int new_h = h_.find(v->getId())->second + neighbor.second->getDivisions();
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
            unsigned int new_h = h_.find(v->getId())->second + neighbor.second->getDivisions();
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

bool LPAStar::isConstrained(std::shared_ptr<Vertex> vertex, std::shared_ptr<Edge> edge, unsigned int current_time,
                                       std::pair<std::unordered_map<std::shared_ptr<Vertex>, std::vector<Constraint>>, std::unordered_map<std::shared_ptr<Edge>, std::vector<Constraint>>> &constraints)
{
  auto constraint = constraints.second.find(edge);
  auto constrained_vertex1 = constraints.first.find(vertex);
  double continuous_current_time = toContinuous(current_time);

  if (edge)
  {
    bool edge_constrained = constraint != constraints.second.end();
    if (edge_constrained)
    {
      for (auto c : constraint->second)
      {
        edge_constrained = ((c.time_step >= (continuous_current_time - (timestep_ / 2.0)) && c.time_step <= (continuous_current_time + edge->getTraversalTime() + (timestep_ / 2.0))) ||
                            (c.time_step <= -1.0 * (continuous_current_time - (timestep_ / 2.0)) && c.time_step >= -1.0 * (continuous_current_time + edge->getTraversalTime() + (timestep_ / 2.0))));
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
        vertex_constrained1 = (c.time_step >= (continuous_current_time + edge->getTraversalTime() - (timestep_ / 2.0)) && c.time_step <= (continuous_current_time + edge->getTraversalTime() + (timestep_ / 2.0))) ||
                             (c.time_step <= -1.0 * (continuous_current_time + edge->getTraversalTime() - timestep_ / 2.0) && c.time_step >= -1.0 * (continuous_current_time + edge->getTraversalTime() + timestep_ / 2.0));
        if (vertex_constrained1)
        {
          // ROS_INFO("IS VERTEX1 CONSTRAINED, c.timestep: %f, current_time: %f, trav_time: %f", c.time_step, current_time, edge->getDivisions());
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
        vertex_constrained2 = (c.time_step >= (continuous_current_time - (timestep_ / 2.0)) && c.time_step <= (continuous_current_time + (timestep_ / 2.0))) ||
                             (c.time_step <= -1.0 * (continuous_current_time - timestep_ / 2.0) && c.time_step >= -1.0 * (continuous_current_time + timestep_ / 2.0));
        if (vertex_constrained2)
        {
          // ROS_INFO("IS VERTEX1 CONSTRAINED, c.timestep: %f, current_time: %f, trav_time: %f", c.time_step, current_time, edge->getDivisions());
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
        vertex_constrained1 = (c.time_step >= (continuous_current_time + (0.5 * timestep_)) && c.time_step <= (continuous_current_time + (1.5 * timestep_))) ||
                             (c.time_step <= -1.0 * (continuous_current_time + 0.5 * timestep_) && c.time_step >= -1.0 * (continuous_current_time + 1.5 * timestep_));
        vertex_constrained1 = vertex_constrained1 || (c.time_step >= (continuous_current_time - (0.5 * timestep_)) && c.time_step <= (continuous_current_time + (0.5 * timestep_))) ||
                                                    (c.time_step <= -1.0 * (continuous_current_time - 0.5 * timestep_) && c.time_step >= -1.0 * (continuous_current_time + 0.5 * timestep_));
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

bool LPAStar::isValid(std::shared_ptr<Vertex> vertex, std::shared_ptr<Edge> edge)
{
  return edge->isValid() && vertex->isValid();
  ;
}

double LPAStar::round(double in)
{
  if (in != std::numeric_limits<double>::infinity())
    return static_cast<int>(std::round(in / timestep_)) * timestep_;
  return in;
}

unsigned int LPAStar::toDiscrete(double in)
{
  if (in != std::numeric_limits<double>::infinity())
    return static_cast<int>(std::round(in / timestep_));
  return std::numeric_limits<unsigned int>::max();
}

double LPAStar::toContinuous(unsigned int in)
{
  if (in != std::numeric_limits<unsigned int>::max())
    return round(in * timestep_);
  return std::numeric_limits<double>::infinity();
}