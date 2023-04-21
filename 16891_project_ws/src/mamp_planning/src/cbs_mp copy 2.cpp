#include "mamp_planning/cbs_mp.hpp"

CBSMP::CBSMP()
{
  initialized_ = false;
  S_ = 1;
  timer_ = n_.createTimer(ros::Duration(1.0 / PLANNER_RATE), &CBSMP::timerCallback, this);
  alpha_ = 0.05;
  X_ = 0.95;
  // mamp_helper_ = std::make_shared<MAMP_Helper>(world_planning_scene, timestep);
}

void CBSMP::initialize(std::vector<std::shared_ptr<Agent>> &agents, std::string &world_planning_scene, double timestep)
{
  mamp_helper_ = std::make_shared<MAMP_Helper>(world_planning_scene, timestep);
  // initialize all agents with planning scenes
  for (auto a : agents)
  {
    agents_.insert({a->getID(), a});
  }

  for (auto a : agents_)
  {
    a.second->getPRM()->buildPRM();

    ROS_INFO("PRM Size: %ld", a.second->getPRM()->PRMgraph_.size());
    a.second->computeSingleAgentPath();
  }
  replanCBS();
  initialized_ = true;
}

void CBSMP::timerCallback(const ros::TimerEvent &)
{
  if (!initialized_)
  {
    ROS_INFO("In Timer! Not Initialized Yet!");
    return;
  }
  ROS_INFO("In Timer!");
}

std::vector<std::shared_ptr<Agent>> CBSMP::getAgents()
{
  std::vector<std::shared_ptr<Agent>> agents;
  for (auto a : agents_)
  {
    agents.push_back(a.second);
  }
  return agents;
}

bool CBSMP::shouldResample(unsigned int N)
{
  double p = 1.0 - pow(X_, alpha_ * N / S_);
  return ((double)rand()/(double)RAND_MAX) < p;
}

void CBSMP::printPaths(std::shared_ptr<CTNode> node)
{
  ROS_INFO("Number of paths: %ld", node->getPaths().size());
  for (auto p : node->getPaths())
  {
    ROS_INFO("Path for %s", p.first.c_str());
    int t = 0;
    for (auto v : p.second)
    {
      std::cout << t << ": ";
      for (int i = 0; i < v->getJointPos().size(); ++i)
      {
        std::cout << v->getJointPos()[i] << ", ";
      }
      ++t;
      std::cout << std::endl;
    }
  }
}

void CBSMP::printConstraints(std::vector<Constraint> constraints)
{
  for (auto c : constraints)
  {
    if (c.is_vertex_constraint)
    {
      ROS_INFO("Vertex Constraint: Agent: %s, Time: %f", c.agent_id.c_str(), c.time_step);
      for (double j : c.joint_pos_vertex->getJointPos())
      {
        ROS_INFO("%f, ", j);
      }
    }
    else
    {
      ROS_INFO("Edge Constraint: Agent: %s, Time: %f", c.agent_id.c_str(), c.time_step);
      auto joint_positions = *(c.joint_pos_edge->getVertexPositions());
      for (auto vec : joint_positions)
      {
        ROS_INFO("Vertex: ");
        for (double j : vec)
        {
          ROS_INFO("%f, ", j);
        }
      }
    }
  }
}

void CBSMP::printCollision(Collision c)
{
  ROS_INFO("Collision:");
  ROS_INFO("Agent 1: %s, time: %f", c.agent_id1.c_str(), c.timestep);
  if (c.location1_is_vertex)
  {
    ROS_INFO("Location 1 is Vertex:");
    for (double j : c.location1_vertex->getJointPos())
    {
      ROS_INFO("%f, ", j);
    }
  }
  else
  {
    ROS_INFO("Location 1 is Edge:");
    auto joint_positions = *(c.location1->getVertexPositions());
    for (auto vec : joint_positions)
    {
      ROS_INFO("Vertex of Edge: ");
      for (double j : vec)
      {
        ROS_INFO("%f, ", j);
      }
    }
  }
  ROS_INFO("Agent 2: %s, time: %f", c.agent_id2.c_str(), c.timestep);
  if (c.location2_is_vertex)
  {
    ROS_INFO("Location 2 is Vertex:");
    for (double j : c.location2_vertex->getJointPos())
    {
      ROS_INFO("%f, ", j);
    }
  }
  else
  {
    ROS_INFO("Location 2 is Edge:");
    auto joint_positions = *(c.location2->getVertexPositions());
    for (auto vec : joint_positions)
    {
      ROS_INFO("Vertex of Edge: ");
      for (double j : vec)
      {
        ROS_INFO("%f, ", j);
      }
    }
  }
}


bool CBSMP::replanCBS()
{
  auto start = std::chrono::high_resolution_clock::now();
  unsigned int node_id = 0;
  unsigned int N = 0;
  std::shared_ptr<CTNode> root = std::make_shared<CTNode>(++node_id, agents_, mamp_helper_);
  for (auto a : agents_)
  {
    root->getPaths().insert({a.first, a.second->getDiscretizedPath()});
  }

  // ROS_INFO("Number of paths: %ld", root->getPaths().size());
  root->detectCollisions();
  // ROS_INFO("Number of paths: %ld", root->getPaths().size());
  root->computeCost();
  // ROS_INFO("Number of paths: %ld", root->getPaths().size());
  open_list_.insert(root->getComparisonTuple(), root);
  int iteration=0;
  while (open_list_.size() > 0)
  {
    auto end = std::chrono::high_resolution_clock::now();
    std::chrono::duration<double> diff = end - start;
    if (diff.count() > 1000)
    {
      ROS_INFO("Failed to find path");
      return false;
    }

    
    if (shouldResample(N))
    {
      // TODO: resample routine
      ROS_INFO("Resampling now!!!");
      auto a = getAgents();
      #ifdef MP_EN
        omp_set_num_threads(MP_PROC_NUM);
        #pragma omp parallel for
      #endif
      for (int i = 0; i < a.size(); ++i)
      {
        a[i]->getPRM()->expandPRM();
        a[i]->computeSingleAgentPath();
      }
      open_list_.clear();
      iteration = 0;
      N = 0;
      node_id = 0;
      root = std::make_shared<CTNode>(++node_id, agents_, mamp_helper_);
      for (auto a : agents_)
      {
        root->getPaths().insert({a.first, a.second->getDiscretizedPath()});
      }
      root->detectCollisions();
      root->computeCost();
      open_list_.insert(root->getComparisonTuple(), root);
      ++S_;
    }
    
    ROS_ERROR("~~~~~~~ CBS Iteration %d ~~~~~~~", iteration++);
    std::shared_ptr<CTNode> node = open_list_.pop().second;
    ++N;

    ROS_INFO("Number of constraints: %ld", node->getConstraints().size());
    ROS_INFO("Number of collisions: %ld", node->numCollisions());
    ROS_INFO("Cost of Node: %f", node->getCost());
    ROS_INFO("Node Id: %d", node->getId());
    // printConstraints(node->getConstraints());
    // ++N;
    if (node->numCollisions() == 0)
    {
      // TODO: publish path and return
      // ROS_INFO("No collisions!!!");
      // ROS_INFO("Number of Constraints: %ld", node->getConstraints().size());
      // printPaths(node);
      end = std::chrono::high_resolution_clock::now();
      diff = end - start;
      ROS_INFO("Runtime: %f", diff.count());
      agents_ = node->getAgents();
      ROS_INFO("Path Cost of Node: %f", node->getCost());
      size_t prm_size = 0;
      for (auto a : agents_)
      {
        prm_size += a.second->getPRM()->PRMgraph_.size();
      }
      ROS_INFO("Total Nodes in All PRMs: %ld", prm_size);

      return true;
    }
    Collision c = node->getNextCollision();
    // ROS_INFO("Collision: Agent 1 - %s, Agent 2 - %s, IsVertex: %d, %d, time: %f", c.agent_id1.c_str(), c.agent_id2.c_str(), c.location1_is_vertex, c.location2_is_vertex, c.timestep);
    std::vector<Constraint> constraints = MAMP_Helper::resolveCollision(c);
    // printCollision(c);
    // printConstraints(constraints);
    std::vector<std::shared_ptr<CTNode>> new_nodes {std::make_shared<CTNode>(++node_id, node), std::make_shared<CTNode>(++node_id, node)};
    std::vector<bool> succ {false, false};
    #ifdef MP_EN
        // ROS_INFO("Using OMP");
        omp_set_num_threads(MP_PROC_NUM);
        #pragma omp parallel for
    #endif
    for (int i = 0; i < new_nodes.size(); ++i)
    {
      new_nodes[i]->addConstraint(constraints[i]);
      succ[i] = new_nodes[i]->getAgents().find(constraints[i].agent_id)->second->computeSingleAgentPath(
      MAMP_Helper::getConstraintsForAgent(new_nodes[i]->getConstraints(), constraints[i].agent_id), new_nodes[i]->getMaxConstraintTime());
      if (succ[i])
      {
        new_nodes[i]->getPaths().erase(constraints[i].agent_id);
        new_nodes[i]->getPaths().insert({constraints[i].agent_id, new_nodes[i]->getAgents().find(constraints[i].agent_id)->second->getDiscretizedPath()});
        new_nodes[i]->detectCollisions();
        new_nodes[i]->computeCost();
        // ROS_INFO("Number of collisions: %ld", new_nodes[i]->numCollisions());
        // ROS_INFO("Cost: %f", new_nodes[i]->getCost());
        // ROS_INFO("Node Id: %d", new_nodes[i]->getId());
        open_list_.insert(new_nodes[i]->getComparisonTuple(), new_nodes[i]);
      }
    }

    // std::shared_ptr<CTNode> n1 = std::make_shared<CTNode>(++node_id, node);
    // std::shared_ptr<CTNode> n2 = std::make_shared<CTNode>(++node_id, node);
    // ROS_INFO("constraints[0].agent_id.c_str(): %s", constraints[0].agent_id.c_str());
    // ROS_INFO("constraints[0].time_step: %f", constraints[0].time_step);
    // ROS_INFO("constraints[0].joint_pos_edge->getTraversalTime(): %f", constraints[0].joint_pos_edge->getTraversalTime());
    // ROS_INFO("Constraint 0: %s, %f, %f", constraints[0].agent_id.c_str(), constraints[0].time_step, constraints[0].joint_pos_edge->getTraversalTime());
    // ROS_INFO("constraints[1].agent_id.c_str(): %s", constraints[1].agent_id.c_str());
    // ROS_INFO("constraints[1].time_step: %f", constraints[1].time_step);
    // ROS_INFO("constraints[1].joint_pos_edge->getTraversalTime(): %f", constraints[1].joint_pos_edge->getTraversalTime());
    // ROS_INFO("Constraint 1: %s, %f, %f", constraints[1].agent_id.c_str(), constraints[1].time_step, constraints[1].joint_pos_edge->getTraversalTime());
    // printConstraints(n1->getConstraints());
    // n1->addConstraint(constraints[0]);
    // ROS_INFO("Full constraint list for n1");
    // printConstraints(n1->getConstraints());
    // n2->addConstraint(constraints[1]);
    // TODO: Get n1 and n2 to recompute paths for specific agents affected
    // ROS_INFO("n1's Agent path length: %ld", n1->getAgents().find(constraints[0].agent_id)->second->getDiscretizedPath().size());
    // for (size_t i = 35; i < 38; ++i)
    // {
    //   auto v = n1->getAgents().find(constraints[0].agent_id)->second->getDiscretizedPath()[i];
    //   ROS_INFO("Vertex %ld %d", i, v->getPRMEdge() != nullptr);
    //   for (double j : v->getJointPos())
    //   {
    //     ROS_INFO("%f,", j);
    //   }
    // }
    // for (size_t i = 0; i < 15; ++i)
    // {
    //   auto v = n1->getAgents().find(constraints[1].agent_id)->second->getDiscretizedPath()[i];
    //   ROS_INFO("Vertex %ld", i);
    //   for (double j : v->getJointPos())
    //   {
    //     ROS_INFO("%f,", j);
    //   }
    // }
    // bool succ1 = n1->getAgents().find(constraints[0].agent_id)->second->computeSingleAgentPath(
    //   MAMP_Helper::getConstraintsForAgent(n1->getConstraints(), constraints[0].agent_id), n1->getMaxConstraintTime());
    
    // bool succ2 = n2->getAgents().find(constraints[1].agent_id)->second->computeSingleAgentPath(
    //   MAMP_Helper::getConstraintsForAgent(n2->getConstraints(), constraints[1].agent_id), n2->getMaxConstraintTime());
    
    // insert n1 and n2 into open list of cbs
    // if (succ1)
    // {
    //   n1->getPaths().erase(constraints[0].agent_id);
    //   n1->getPaths().insert({constraints[0].agent_id, n1->getAgents().find(constraints[0].agent_id)->second->getDiscretizedPath()});
    //   // ROS_INFO("n1's Agent path length after update: %ld", n1->getAgents().find(constraints[0].agent_id)->second->getDiscretizedPath().size());
    //   // for (size_t i = 35; i < 38; ++i)
    //   // {
    //   //   auto v = n1->getAgents().find(constraints[0].agent_id)->second->getDiscretizedPath()[i];
    //   //   ROS_INFO("Vertex %ld %d", i, v->getPRMEdge() != nullptr);
    //   //   for (double j : v->getJointPos())
    //   //   {
    //   //     ROS_INFO("%f,", j);
    //   //   }
    //   // }
    //   // for (size_t i = 0; i < 15; ++i)
    //   // {
    //   //   auto v = n1->getAgents().find(constraints[1].agent_id)->second->getDiscretizedPath()[i];
    //   //   ROS_INFO("Vertex %ld", i);
    //   //   for (double j : v->getJointPos())
    //   //   {
    //   //     ROS_INFO("%f,", j);
    //   //   }
    //   // }
    //   n1->detectCollisions();
    //   // ROS_INFO("Next collision at %f between %s %s", n1->getNextCollision().timestep, n1->getNextCollision().agent_id1.c_str(), n1->getNextCollision().agent_id2.c_str());
    //   n1->computeCost();
    //   ROS_INFO("Cost of n1: %f", n1->getCost());
    //   open_list_.insert(n1->getComparisonTuple(), n1);
    // }
    // // else
    // // {
    // //   ROS_ERROR("Failed to solve AStar for n1");
    // // }
    // if (succ2)
    // {
    //   n2->getPaths().erase(constraints[1].agent_id);
    //   n2->getPaths().insert({constraints[1].agent_id, n2->getAgents().find(constraints[1].agent_id)->second->getDiscretizedPath()});
    //   n2->detectCollisions();
    //   n2->computeCost();
    //   ROS_INFO("Cost of n2: %f", n2->getCost());
    //   open_list_.insert(n2->getComparisonTuple(), n2);
    // }
    // else
    // {
    //   ROS_ERROR("Failed to solve AStar for n2");
    // }
  }
  // No solution!!!
  return false;
}
