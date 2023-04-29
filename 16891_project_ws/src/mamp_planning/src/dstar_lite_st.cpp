#include "mamp_planning/dstar_lite_st.hpp"

DStarLiteST::DStarLiteST(double timestep)
{
  timestep_ = timestep;
  path_time_ = 0;
  max_roadmap_traversal_time_ = 0;
}

DStarLiteST::DStarLiteST(std::shared_ptr<DStarLiteST> &d)
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

OpenList<std::tuple<double, double, unsigned int, double>, std::tuple<unsigned int, double>, Vertex, hash_tuple::hash<std::tuple<unsigned int, double>>> &DStarLiteST::getOpenList()
{
  return open_list_;
}

std::unordered_map<std::tuple<unsigned int, double>, double, hash_tuple::hash<std::tuple<unsigned int, double>>> &DStarLiteST::getGMap()
{
  return g_;
}

std::unordered_map<std::tuple<unsigned int, double>, double, hash_tuple::hash<std::tuple<unsigned int, double>>> &DStarLiteST::getRHSMap()
{
  return rhs_;
}

std::unordered_map<unsigned int, double> &DStarLiteST::getH()
{
  return h_;
}

double DStarLiteST::getMaxRoadmapTraversalTime()
{
  return max_roadmap_traversal_time_;
}

double DStarLiteST::getTimestep()
{
  return timestep_;
}

double DStarLiteST::getPathTime()
{
  return path_time_;
}

std::set<double> &DStarLiteST::getExpandedGoalTimes()
{
  return expanded_goal_times_;
}

double DStarLiteST::getG(std::tuple<unsigned int, double> vertex)
{
  if (g_.find(vertex) != g_.end())
    return g_[vertex];
  else
    return std::numeric_limits<double>::infinity();
}

void DStarLiteST::setG(std::tuple<unsigned int, double> key, double value)
{
  g_[key] = value;
}

double DStarLiteST::getRHS(std::tuple<unsigned int, double> vertex)
{
  if (rhs_.find(vertex) != rhs_.end())
    return rhs_[vertex];
  else
    return std::numeric_limits<double>::infinity();
}

void DStarLiteST::setRHS(std::tuple<unsigned int, double> key, double value)
{
  rhs_[key] = value;
}

std::tuple<double, double> DStarLiteST::calculateKey(std::tuple<unsigned int, double> vertex)
{
  double k1 = std::min(getG(vertex), getRHS(vertex)) + h_[std::get<0>(vertex)];
  double k2 = std::min(getG(vertex), getRHS(vertex));
  k1 = round(k1);
  k2 = round(k2);
  return std::make_tuple(k1, k2);
}

void DStarLiteST::initialize(std::shared_ptr<Vertex> start, std::shared_ptr<Vertex> goal)
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

void DStarLiteST::updateVertex(std::tuple<unsigned int, double> vertex, std::shared_ptr<Vertex> vertex_ptr, std::shared_ptr<Vertex> goal)
{
  if (round(getG(vertex)) != round(getRHS(vertex)))
  {
    // insert the vertex into the open list
    // ROS_INFO("key: %f, %f, id: %d, time: %f", std::get<0>(calculateKey(vertex)), std::get<1>(calculateKey(vertex)), vertex_ptr->getId(), std::get<1>(vertex));
    open_list_.insert(std::tuple_cat(calculateKey(vertex), vertex), vertex, vertex_ptr);
    if (vertex_ptr->getId() == goal->getId())
      expanded_goal_times_.erase(round(std::get<1>(vertex)));
    // key_map_[vertex] = key;
  }
  else if (round(getG(vertex)) == round(getRHS(vertex)))
  {
    // remove the vertex from the open list
    // ROS_INFO("REMOVED: %d, t: %f", std::get<0>(vertex), std::get<1>(vertex));
    open_list_.remove(vertex);
    if (vertex_ptr->getId() == goal->getId() && getG(vertex) != std::numeric_limits<double>::infinity())
      expanded_goal_times_.insert(round(std::get<1>(vertex)));
  }
}

// bool DStarLiteST::computeWaypointPaths(std::vector<std::shared_ptr<Vertex>> waypoints,
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

// void DStarLiteST::clearExpandedGoals(std::shared_ptr<Vertex> goal)
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

bool DStarLiteST::computePRMPath(std::shared_ptr<Vertex> start, std::shared_ptr<Vertex> goal,
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
    if (c.is_vertex_constraint)
    {
      // ROS_INFO("current time vertex: %f", c.time_step);
      // vertex constraint
      // all predecessors
      if (c.joint_pos_vertex->getId() != start->getId() && round(c.time_step) != 0)
      {
        setRHS(std::make_tuple(c.joint_pos_vertex->getId(), round(c.time_step)), std::numeric_limits<double>::infinity());
        // ROS_INFO("Setting RHS to inf, ID: %d, Time: %f", c.joint_pos_vertex->getId(), round(c.time_step));
        updateVertex(std::make_tuple(c.joint_pos_vertex->getId(), round(c.time_step)), c.joint_pos_vertex, goal);
      }

      // all successors
      // All PRM Neighbor edges
      for (auto succ : c.joint_pos_vertex->getEdges())
      {
        if (round(getRHS(std::make_tuple(succ.first->getId(), round(c.time_step + succ.second->getTraversalTime())))) == round(succ.second->getTraversalTime() + getG(std::make_tuple(c.joint_pos_vertex->getId(), round(c.time_step)))))
        {
          // Line 46, for all predecessors (backwards)
          double min_rhs = std::numeric_limits<double>::infinity();
          for (auto pred : succ.first->getEdges())
          {
            if (round(c.time_step + succ.second->getTraversalTime() - pred.second->getTraversalTime()) >= 0 &&
                !isConstrained(succ.first, pred.second, c.time_step + succ.second->getTraversalTime() - pred.second->getTraversalTime(), constraints) &&
                round(min_rhs) > round(pred.second->getTraversalTime() + getG(std::make_tuple(pred.first->getId(), round(c.time_step + succ.second->getTraversalTime() - pred.second->getTraversalTime())))))
            {
              min_rhs = round(pred.second->getTraversalTime() + getG(std::make_tuple(pred.first->getId(), round(c.time_step + succ.second->getTraversalTime() - pred.second->getTraversalTime()))));
              // ROS_INFO("Got MinRHS from Pred ID %d at time %f to RHS: %f", pred.first->getId(), c.time_step + succ.second->getTraversalTime() - pred.second->getTraversalTime(), min_rhs);
              // ROS_INFO("Constrained: %d", isConstrained(succ.first, pred.second, c.time_step + succ.second->getTraversalTime() - pred.second->getTraversalTime(), constraints));
            }
          }
          if (!isConstrained(succ.first, nullptr, c.time_step + succ.second->getTraversalTime() - timestep_, constraints) &&
              round(min_rhs) > round(timestep_ + getG(std::make_tuple(succ.first->getId(), round(c.time_step + succ.second->getTraversalTime() - timestep_)))))
          {
            min_rhs = round(timestep_ + getG(std::make_tuple(succ.first->getId(), round(c.time_step + succ.second->getTraversalTime() - timestep_))));
          }
          // ROS_INFO("Setting Successor ID %d at time %f to RHS: %f", succ.first->getId(), c.time_step + succ.second->getTraversalTime(), min_rhs);
          setRHS(std::make_tuple(succ.first->getId(), round(c.time_step + succ.second->getTraversalTime())), min_rhs);
        }
        updateVertex(std::make_tuple(succ.first->getId(), round(c.time_step + succ.second->getTraversalTime())), succ.first, goal);
      }
      // Staying in same spot edge
      if (round(getRHS(std::make_tuple(c.joint_pos_vertex->getId(), round(c.time_step + timestep_)))) == round(timestep_ + getG(std::make_tuple(c.joint_pos_vertex->getId(), round(c.time_step)))))
      {
        // Line 46, for all predecessors (backwards)
        double min_rhs = std::numeric_limits<double>::infinity();
        for (auto pred : c.joint_pos_vertex->getEdges())
        {
          if (round(c.time_step + timestep_ - pred.second->getTraversalTime()) >= 0 &&
              !isConstrained(c.joint_pos_vertex, pred.second, c.time_step + timestep_ - pred.second->getTraversalTime(), constraints) &&
              round(min_rhs) > round(pred.second->getTraversalTime() + getG(std::make_tuple(pred.first->getId(), round(c.time_step + timestep_ - pred.second->getTraversalTime())))))
          {
            min_rhs = round(pred.second->getTraversalTime() + getG(std::make_tuple(pred.first->getId(), round(c.time_step + timestep_ - pred.second->getTraversalTime()))));
          }
        }
        // ROS_INFO("Setting Successor ID %d at time %f to RHS: %f", c.joint_pos_vertex->getId(), round(c.time_step + timestep_), min_rhs);
        setRHS(std::make_tuple(c.joint_pos_vertex->getId(), round(c.time_step + timestep_)), min_rhs);
      }
      updateVertex(std::make_tuple(c.joint_pos_vertex->getId(), round(c.time_step + timestep_)), c.joint_pos_vertex, goal);
    }
    else
    {
      // Edge constraint
      double c_old = c.joint_pos_edge->getTraversalTime();
      for (double current_time = c.time_step - c.joint_pos_edge->getTraversalTime(); round(current_time) <= round(c.time_step); current_time += timestep_)
      {
        // ROS_INFO("current time edge: %f, timestep: %f, trav time: %f", current_time, c.time_step, c.joint_pos_edge->getTraversalTime());
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
          if (round(getRHS(std::make_tuple(c.joint_pos_edge->ordered_vertices_[j]->getId(), round(current_time + c.joint_pos_edge->getTraversalTime())))) ==
              round(c_old + getG(std::make_tuple(c.joint_pos_edge->ordered_vertices_[k]->getId(), round(current_time)))))
          {
            double min_rhs = std::numeric_limits<double>::infinity();
            for (auto pred : c.joint_pos_edge->ordered_vertices_[j]->getEdges())
            {
              if (round(current_time + c.joint_pos_edge->getTraversalTime() - pred.second->getTraversalTime()) >= 0 &&
                  !isConstrained(c.joint_pos_edge->ordered_vertices_[j], pred.second, current_time + c.joint_pos_edge->getTraversalTime() - pred.second->getTraversalTime(), constraints) &&
                  round(min_rhs) > round(pred.second->getTraversalTime() + getG(std::make_tuple(pred.first->getId(), round(current_time + c.joint_pos_edge->getTraversalTime() - pred.second->getTraversalTime())))))
              {
                min_rhs = round(pred.second->getTraversalTime() + getG(std::make_tuple(pred.first->getId(), round(current_time + c.joint_pos_edge->getTraversalTime() - pred.second->getTraversalTime()))));
              }
              // else if (isConstrained(c.joint_pos_edge->ordered_vertices_[j], pred.second, current_time + c.joint_pos_edge->getTraversalTime() - pred.second->getTraversalTime(), constraints))
              // {
              //   ROS_INFO("Constrained, ID: %d, t: %f", c.joint_pos_edge->ordered_vertices_[j]->getId(), current_time + c.joint_pos_edge->getTraversalTime() - pred.second->getTraversalTime());
              // }
            }
            if (!isConstrained(c.joint_pos_edge->ordered_vertices_[j], nullptr, current_time + c.joint_pos_edge->getTraversalTime() - timestep_, constraints) &&
                round(min_rhs) > round(timestep_ + getG(std::make_tuple(c.joint_pos_edge->ordered_vertices_[j]->getId(), round(current_time + c.joint_pos_edge->getTraversalTime() - timestep_)))))
            {
              min_rhs = round(timestep_ + getG(std::make_tuple(c.joint_pos_edge->ordered_vertices_[j]->getId(), round(current_time + c.joint_pos_edge->getTraversalTime() - timestep_))));
            }
            // else if (isConstrained(c.joint_pos_edge->ordered_vertices_[j], nullptr, current_time + c.joint_pos_edge->getTraversalTime() - timestep_, constraints))
            // {
            //   ROS_INFO("Constrained, ID: %d, t: %f", c.joint_pos_edge->ordered_vertices_[j]->getId(), current_time + c.joint_pos_edge->getTraversalTime() - timestep_);
            // }
            // ROS_INFO("Setting Successor ID %d at time %f to RHS: %f", c.joint_pos_edge->ordered_vertices_[j]->getId(), round(current_time + c.joint_pos_edge->getTraversalTime()), min_rhs);

            setRHS(std::make_tuple(c.joint_pos_edge->ordered_vertices_[j]->getId(), round(current_time + c.joint_pos_edge->getTraversalTime())), min_rhs);
          }
          updateVertex(std::make_tuple(c.joint_pos_edge->ordered_vertices_[j]->getId(), round(current_time + c.joint_pos_edge->getTraversalTime())), c.joint_pos_edge->ordered_vertices_[j], goal);
        }
      }
    }
  }
  return computeShortestPath(start, goal, constraints, max_constraint_time);
}

bool DStarLiteST::shortestPathSearchCondition(std::shared_ptr<Vertex> start, std::shared_ptr<Vertex> goal,
                         std::pair<std::unordered_map<std::shared_ptr<Vertex>, std::vector<Constraint>>, std::unordered_map<std::shared_ptr<Edge>, std::vector<Constraint>>> &constraints,
                         double max_constraint_time)
{
  // ROS_INFO("open_list_.size(): %ld", open_list_.size());
  // ROS_INFO("g_.size(): %ld", g_.size());
  // ROS_INFO("rhs_.size(): %ld", rhs_.size());

  if (open_list_.size() == 0)
  {
    ROS_ERROR("No Path!");
    return false;
  }
  // ROS_INFO("Number of expanded_goal_times_: %ld", expanded_goal_times_.size());
  for (double goal_time : expanded_goal_times_)
  {
    // ROS_INFO("goal_time: %f", goal_time);
    
    auto top_key = std::get<0>(open_list_.top());
    std::tuple<double, double> key = std::make_tuple(round(std::get<0>(top_key)), round(std::get<1>(top_key)));
    std::tuple<unsigned int, double> goal_tuple = std::make_tuple(goal->getId(), goal_time);
      // ROS_INFO("top key: %f, %f", std::get<0>(top_key), std::get<1>(top_key));
      // ROS_INFO("goal key: %f, %f", std::get<0>(calculateKey(goal_tuple)), std::get<1>(calculateKey(goal_tuple)));
    if (key >= calculateKey(goal_tuple) && round(getRHS(goal_tuple)) <= round(getG(goal_tuple)) && round(goal_time) >= round(max_constraint_time))
    {
      
      // ROS_INFO("top key greater than goal key: %d", key >= calculateKey(goal_tuple));
      // ROS_INFO("first vs first: %d", round(std::get<0>(top_key)) >= std::get<0>(calculateKey(goal_tuple)));
      // ROS_INFO("second vs second: %d", std::get<1>(top_key) >= std::get<1>(calculateKey(goal_tuple)));
      // ROS_INFO("getRHS(goal_tuple): %f", getRHS(goal_tuple));
      // ROS_INFO("getG(goal_tuple): %f", getG(goal_tuple));
      path_time_ = goal_time;
      // ROS_INFO("Found a good goal time");
      // std::vector<std::shared_ptr<Vertex>> test = getPRMPath(start, goal, constraints);
      // if (test.size() == 0 || test[0]->getId() != start->getId())
      // {
      //   continue;
      // }
      return false;
    }
  }
  return true;
}

bool DStarLiteST::computeShortestPath(std::shared_ptr<Vertex> start, std::shared_ptr<Vertex> goal,
                         std::pair<std::unordered_map<std::shared_ptr<Vertex>, std::vector<Constraint>>, std::unordered_map<std::shared_ptr<Edge>, std::vector<Constraint>>> &constraints,
                         double max_constraint_time)
{
  double max_time = round(max_constraint_time + max_roadmap_traversal_time_);
  while(shortestPathSearchCondition(start, goal, constraints, max_constraint_time))
  {
    ++num_expansions_;
    // ROS_INFO("Updating");
    auto top = open_list_.top();
    std::tuple<double, double> k_old = std::make_tuple(round(std::get<0>(std::get<0>(top))), std::get<1>(std::get<0>(top)));

    std::tuple<unsigned int, double> u_tuple = std::get<1>(top);
    if (round(std::get<1>(u_tuple)) > max_time)
      continue;
    std::shared_ptr<Vertex> u = std::get<2>(top);
    // ROS_INFO("old key: %f, %f, id: %d, time: %f", std::get<0>(k_old), std::get<1>(k_old), u->getId(), std::get<1>(u_tuple));
    std::tuple<double, double> k_new = calculateKey(u_tuple);
    // ROS_INFO("new key: %f, %f, id: %d, time: %f", std::get<0>(k_new), std::get<1>(k_new), u->getId(), std::get<1>(u_tuple));

    if (k_old < k_new)
    {
      // ROS_INFO("Update key");
      open_list_.insert(std::tuple_cat(k_new, u_tuple), u_tuple, u);
      updateVertex(u_tuple, u, goal);
      // double u_time = round(std::get<1>(u_tuple));
      // if (u->getId() == goal->getId())
      //   expanded_goal_times_.erase(u_time);
    }
    else if (round(getG(u_tuple)) > round(getRHS(u_tuple)))
    {
      setG(u_tuple, round(getRHS(u_tuple)));
      // ROS_INFO("Expand key");
      // open_list_.remove(u_tuple);
      updateVertex(u_tuple, u, goal);
      double u_time = round(std::get<1>(u_tuple));
      // ROS_INFO("expanded id: %d, time: %f, g: %f", u->getId(), u_time, getG(u_tuple));
      // if (u->getId() == goal->getId())
      //   expanded_goal_times_.insert(u_time);
      for (auto succ : u->getEdges())
      {
        // if (succ.first->getId() == goal->getId())
        // {
        //   ROS_INFO("Just modified predecessor to goal");
        //   ROS_INFO("pred time: %f", u_time);
        //   ROS_INFO("goal time: %f", u_time + succ.second->getTraversalTime());
        //   ROS_INFO("pred id: %d", u->getId());
        //   ROS_INFO("goal id: %d", goal->getId());
        // }
        if (round(u_time + succ.second->getTraversalTime()) <= max_time)
        {
          std::tuple<unsigned int, double> succ_tuple = std::make_tuple(succ.first->getId(), round(u_time + succ.second->getTraversalTime()));
          if (!isConstrained(succ.first, succ.second, u_time, constraints) &&
          round(getRHS(succ_tuple)) > round(succ.second->getTraversalTime() + getG(u_tuple)))
          {
            setRHS(succ_tuple, round(succ.second->getTraversalTime() + getG(u_tuple)));
          }
          updateVertex(succ_tuple, succ.first, goal);
          // if (succ.first->getId() == goal->getId())
          // {
          //   ROS_INFO("Updating goal (successor)");
          // }
        }
      }
      if (round(u_time + timestep_) <= max_time)
      {
        std::tuple<unsigned int, double> succ_tuple = std::make_tuple(u->getId(), round(u_time + timestep_));
        if (!isConstrained(u, nullptr, u_time, constraints) &&
        round(getRHS(succ_tuple)) > round(timestep_ + getG(u_tuple)))
        {
          setRHS(succ_tuple, round(timestep_ + getG(u_tuple)));
        }
        updateVertex(succ_tuple, u, goal);
      }
    }
    else
    {
      // ROS_INFO("Reset key");
      double g_old = getG(u_tuple);
      double u_time = round(std::get<1>(u_tuple));
      setG(u_tuple, std::numeric_limits<double>::infinity());
      // if (u->getId() == goal->getId())
      //   expanded_goal_times_.erase(u_time);
      for (auto succ : u->getEdges())
      {
        if (round(u_time + succ.second->getTraversalTime()) <= max_time)
        {
          std::tuple<unsigned int, double> succ_tuple = std::make_tuple(succ.first->getId(), round(u_time + succ.second->getTraversalTime()));
          if (round(getRHS(succ_tuple)) == round(succ.second->getTraversalTime() + g_old))
          {
            // Line 46, for all predecessors (backwards)
            double min_rhs = std::numeric_limits<double>::infinity();
            for (auto pred : succ.first->getEdges())
            {
              std::tuple<unsigned int, double> pred_tuple = std::make_tuple(pred.first->getId(), round(u_time + succ.second->getTraversalTime() - pred.second->getTraversalTime()));
              if (round(u_time + succ.second->getTraversalTime() - pred.second->getTraversalTime()) >= 0 &&
                  !isConstrained(succ.first, pred.second, u_time + succ.second->getTraversalTime() - pred.second->getTraversalTime(), constraints) &&
                  round(min_rhs) > round(pred.second->getTraversalTime() + getG(pred_tuple)))
              {
                min_rhs = round(pred.second->getTraversalTime() + getG(pred_tuple));
              }
            }
            std::tuple<unsigned int, double> pred_tuple = std::make_tuple(succ.first->getId(), round(u_time + succ.second->getTraversalTime() - timestep_));
            if (!isConstrained(succ.first, nullptr, u_time + succ.second->getTraversalTime() - timestep_, constraints) &&
                round(min_rhs) > round(timestep_ + getG(pred_tuple)))
            {
              min_rhs = round(timestep_ + getG(pred_tuple));
            }
            setRHS(succ_tuple, min_rhs);
          }
          updateVertex(succ_tuple, succ.first, goal);
        }
      }
      // Staying in same spot successor
      if (round(u_time + timestep_) <= max_time)
      {
        std::tuple<unsigned int, double> succ_tuple = std::make_tuple(u->getId(), round(u_time + timestep_));
        if (round(getRHS(succ_tuple)) == round(timestep_ + g_old))
        {
          // Line 46, for all predecessors (backwards)
          double min_rhs = std::numeric_limits<double>::infinity();
          for (auto pred : u->getEdges())
          {
            std::tuple<unsigned int, double> pred_tuple = std::make_tuple(pred.first->getId(), round(u_time + timestep_ - pred.second->getTraversalTime()));
            if (round(u_time + timestep_ - pred.second->getTraversalTime()) >= 0 &&
                !isConstrained(u, pred.second, u_time + timestep_ - pred.second->getTraversalTime(), constraints) &&
                round(min_rhs) > round(pred.second->getTraversalTime() + getG(pred_tuple)))
            {
              min_rhs = round(pred.second->getTraversalTime() + getG(pred_tuple));
            }
          }
          std::tuple<unsigned int, double> pred_tuple = std::make_tuple(u->getId(), u_time);
          if (!isConstrained(u, nullptr, u_time, constraints) &&
              round(min_rhs) > round(timestep_ + getG(pred_tuple)))
          {
            min_rhs = round(timestep_ + getG(pred_tuple));
          }
          setRHS(succ_tuple, min_rhs);
        }
        updateVertex(succ_tuple, u, goal);
      }
      
      // same spot, same time, u
      if (round(getRHS(u_tuple)) == round(g_old) && u->getId() != start->getId() && u_time != 0)
      {
        double min_rhs = std::numeric_limits<double>::infinity();
        for (auto pred : u->getEdges())
        {
          std::tuple<unsigned int, double> pred_tuple = std::make_tuple(pred.first->getId(), round(u_time - pred.second->getTraversalTime()));
          if (round(u_time - pred.second->getTraversalTime()) >= 0 &&
              !isConstrained(u, pred.second, u_time - pred.second->getTraversalTime(), constraints) &&
              round(min_rhs) > round(pred.second->getTraversalTime() + getG(pred_tuple)))
          {
            min_rhs = round(pred.second->getTraversalTime() + getG(pred_tuple));
          }
        }
        std::tuple<unsigned int, double> pred_tuple = std::make_tuple(u->getId(), round(u_time - timestep_));
        if (round(u_time - timestep_) >= 0 &&
            !isConstrained(u, nullptr, u_time - timestep_, constraints) &&
            round(min_rhs) > round(timestep_ + getG(pred_tuple)))
        {
          min_rhs = round(timestep_ + getG(pred_tuple));
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

std::vector<std::shared_ptr<Vertex>> DStarLiteST::getPRMPath(std::shared_ptr<Vertex> start, std::shared_ptr<Vertex> goal,
  std::pair<std::unordered_map<std::shared_ptr<Vertex>, std::vector<Constraint>>, std::unordered_map<std::shared_ptr<Edge>, std::vector<Constraint>>> &constraints)
{
  std::vector<std::shared_ptr<Vertex>> prm_path;
  std::shared_ptr<Vertex> v = goal;
  // std::tuple<std::shared_ptr<Vertex>, double> v_time = std::make_tuple(goal, path_time_);
  prm_path.push_back(v);
  double time = round(path_time_);
  // ROS_INFO("Final open_list_.size(): %ld", open_list_.size());
  // for (auto g_value: g_)
  // {
  //   ROS_INFO("id: %d, t: %.20f, g: %f", std::get<0>(g_value.first), std::get<1>(g_value.first), g_value.second);
  //   ROS_INFO("g from getG: %f", getG(g_value.first));
  //   // ROS_INFO("Equals: %d", std::get<1>(g_value.first) == 2.8);
  //   ROS_INFO("hash: %ld", hash_tuple::hash<std::tuple<unsigned int, double>>{}(g_value.first));
  // }
  // ROS_INFO("GOAL RHS: %f, G: %f", getRHS(std::make_tuple(goal->getId(), round(path_time_))), getG(std::make_tuple(goal->getId(), round(path_time_))));
  // bool same = false;
  while (v->getId() != start->getId() || round(time) != 0)
  {
    std::shared_ptr<Vertex> s = v;
    double pred_time = time;
    double f = std::numeric_limits<double>::infinity();
    for (auto pred : v->getEdges())
    {
      // ROS_INFO("Equals: %d", time - pred.second->getTraversalTime() == 2.8);
      // if (isConstrained(pred.first, pred.second, time - pred.second->getTraversalTime(), constraints))
      //   ROS_WARN("SHOULDNT BE HERE");
      // ROS_INFO("v ID: %d, Time: %f, G: %f, RHS: %f", v->getId(), round(time), getG(std::make_tuple(v->getId(), round(time))), getRHS(std::make_tuple(v->getId(), round(time))));
      // ROS_INFO("Trav Time: %f", pred.second->getTraversalTime());
      // ROS_INFO("Double G: %f", getG(std::make_tuple(pred.first->getId(), round(time - pred.second->getTraversalTime() - pred.second->getTraversalTime()))));
      // ROS_INFO("Pred ID: %d, Time: %f, G: %f, RHS: %f", pred.first->getId(), round(time - pred.second->getTraversalTime()), getG(std::make_tuple(pred.first->getId(), round(time - pred.second->getTraversalTime()))), getRHS(std::make_tuple(pred.first->getId(), round(time - pred.second->getTraversalTime()))));
      // ROS_INFO("Constrained: %d", isConstrained(v, pred.second, time - pred.second->getTraversalTime(), constraints));
      if (round(time - pred.second->getTraversalTime()) >= 0 &&
          !isConstrained(v, pred.second, time - pred.second->getTraversalTime(), constraints) &&
          round(f) > round(pred.second->getTraversalTime() + getG(std::make_tuple(pred.first->getId(), round(time - pred.second->getTraversalTime())))))
      {
        // ROS_INFO("Pred ID: %d, Time: %f, G: %f, RHS: %f", pred.first->getId(), round(time - pred.second->getTraversalTime()), getG(std::make_tuple(pred.first->getId(), round(time - pred.second->getTraversalTime()))), getRHS(std::make_tuple(pred.first->getId(), round(time - pred.second->getTraversalTime()))));
        // ROS_INFO("Trav Time: %f", pred.second->getTraversalTime());
        // ROS_INFO("pred time: %f", time - pred.second->getTraversalTime());
        f = round(pred.second->getTraversalTime() + getG(std::make_tuple(pred.first->getId(), round(time - pred.second->getTraversalTime()))));
        s = pred.first;
        pred_time = time - pred.second->getTraversalTime();
      }
    }
    // ROS_INFO("Pred standstill G: %f", getG(std::make_tuple(v->getId(), time - timestep_)));
    // ROS_INFO("Pred ID: %d, Time: %f, G: %f, RHS: %f", v->getId(), time - timestep_, getG(std::make_tuple(v->getId(), time - timestep_)), getRHS(std::make_tuple(v->getId(), time - timestep_)));
    // ROS_INFO("Constrained: %d", isConstrained(v, nullptr, time - timestep_, constraints));
    if (round(time - timestep_) >= 0 &&
        !isConstrained(v, nullptr, time - timestep_, constraints) &&
        round(f) > round(timestep_ + getG(std::make_tuple(v->getId(), round(time - timestep_)))))
    {
      f = round(timestep_ + getG(std::make_tuple(v->getId(), round(time - timestep_))));
      s = v;
      pred_time = time - timestep_;
    }
    // if (f == std::numeric_limits<double>::infinity())
    // {
    //   std::reverse(prm_path.begin(), prm_path.end());
    //   return prm_path;
    // }
    // if (!same)
    //   ROS_INFO("v ID: %d, start ID: %d, time: %f", v->getId(), start->getId(), time);
    // if (v->getId() == s->getId())
    //   same = true;
    v = s;
    time = round(pred_time);
    prm_path.push_back(v);
  }
  // while (v != start || time != 0)
  // {
  //   v_time = parent_map_.find(v_time)->second;
  //   v = std::get<0>(v_time);
  //   time = std::get<1>(v_time);
  //   prm_path.push_back(v);
  // }
  std::reverse(prm_path.begin(), prm_path.end());
  // ROS_INFO("Returning a path from Dstar");
  return prm_path;
}

unsigned int DStarLiteST::getNumExpansions()
{
  return num_expansions_;
}

double DStarLiteST::computeHeuristics(std::shared_ptr<Vertex> goal)
{
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
            double new_h = h_.find(v->getId())->second + neighbor.second->getTraversalTime();
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
            double new_h = h_.find(v->getId())->second + neighbor.second->getTraversalTime();
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

bool DStarLiteST::isConstrained(std::shared_ptr<Vertex> vertex, std::shared_ptr<Edge> edge, double current_time,
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
        edge_constrained = ((c.time_step >= (current_time - (timestep_ / 2.0)) && c.time_step <= (current_time + edge->getTraversalTime() + (timestep_ / 2.0))) ||
                            (c.time_step <= -1.0 * (current_time - (timestep_ / 2.0)) && c.time_step >= -1.0 * (current_time + edge->getTraversalTime() + (timestep_ / 2.0))));
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

bool DStarLiteST::isValid(std::shared_ptr<Vertex> vertex, std::shared_ptr<Edge> edge)
{
  return edge->isValid() && vertex->isValid();
  ;
}

double DStarLiteST::round(double in)
{
  if (in != std::numeric_limits<double>::infinity())
    return static_cast<int>(std::round(in / timestep_)) * timestep_;
  return in;
}