#include "mamp_planning/cbs_mp_dstar_lite_st.hpp"

CBSMPDStarLiteST::CBSMPDStarLiteST()
{
  initialized_ = false;
  S_ = 1;
  timer_ = n_.createTimer(ros::Duration(1.0 / PLANNER_RATE), &CBSMPDStarLiteST::timerCallback, this);
  alpha_ = 0.05;
  X_ = 0.95;
  // mamp_helper_ = std::make_shared<MAMP_Helper>(world_planning_scene, timestep);
}

void CBSMPDStarLiteST::initialize(std::vector<std::shared_ptr<Agent>> &agents, std::string &world_planning_scene, double timestep)
{

  mamp_helper_ = std::make_shared<MAMP_Helper>(world_planning_scene, timestep);

  // Pass on each agent's combined ACM to the mamp helper
  std::shared_ptr<collision_detection::AllowedCollisionMatrix> multi_robot_acm;
  multi_robot_acm = std::make_shared<collision_detection::AllowedCollisionMatrix>(mamp_helper_->getPlanningScene()->getAllowedCollisionMatrix());
  for (std::shared_ptr<Agent> a : agents)
  {
    // ROS_WARN("Size for this agent was %ld", a->getPRM()->getAcm()->getSize());
    std::vector<std::string> entries;
    a->getPRM()->getAcm()->getAllEntryNames(entries);
    // for (std::string entry : entries)
    // {
    //   ROS_WARN("This was an entry: %s", entry.c_str());
    // }


    // The following is horribly inefficient but doing it because it only needs to be done once
    for (auto entry1 : entries)
    {
      for (auto entry2 : entries)
      {
        collision_detection::AllowedCollision::Type entry_type = collision_detection::AllowedCollision::ALWAYS;
        if (a->getPRM()->getAcm()->getAllowedCollision(entry1, entry2, entry_type))
        {
          // ROS_WARN("This was an entry1, entry2: %s %s", entry1.c_str(), entry2.c_str());
          // ROS_WARN("yeah I'm in.");

          // Adapt the ACM to have entries correctly for the different agents.
          std::string entry1_rename;
          std::string entry2_rename;

          if (std::strcmp(entry1.c_str(), "base") != 0)
          {

            if (std::strcmp(entry1.substr(0,3).c_str(), "she") == 0)
            {
              // Assume the shelf can only hit an arm here
              entry1_rename = entry1.substr(0,6) + a->getID().substr(4, 1); // THIS WILL BREAK IF WE HAVE DOUBLE DIGIT AGENTS
            }
            else
            {
              entry1_rename = a->getID().substr(0, 5) + entry1.substr(5); // THIS WILL BREAK IF WE HAVE DOUBLE DIGIT AGENTS
            }

          }
          else {entry1_rename = entry1;}

          if (std::strcmp(entry2.c_str(), "base") != 0)
          {

            if (std::strcmp(entry2.substr(0,3).c_str(), "she") == 0)
            {
              // Assume the shelf can only hit an arm here
              entry2_rename = entry2.substr(0,6) + a->getID().substr(4, 1); // THIS WILL BREAK IF WE HAVE DOUBLE DIGIT AGENTS
            }
            else
            {
              entry2_rename = a->getID().substr(0, 5) + entry2.substr(5); // THIS WILL BREAK IF WE HAVE DOUBLE DIGIT AGENTS
            }


          }
          else {entry2_rename = entry2;}


          multi_robot_acm->setEntry(entry1_rename, entry2_rename, true);
        }
      }
    }
    std::vector<std::string> entries_check;
    multi_robot_acm->getAllEntryNames(entries_check);
    ROS_WARN("The complete multi robot ACM has this many entries: %ld", entries_check.size());
    for (auto entry : entries_check)
    {
      ROS_WARN("%s", entry.c_str());
    }
  }
  mamp_helper_->setAcm(multi_robot_acm);

  
  
  // initialize all agents with planning scenes
  for (auto a : agents)
  {
    agents_.insert({a->getID(), a});
  }

  for (auto a : agents_)
  {
    // ROS_INFO("Joint check: %d", a.second->getJointVelLimit().size());
    a.second->getPRM()->buildPRM();

    ROS_INFO("Agent %s PRM Size: %ld", a.first.c_str(), a.second->getPRM()->PRMgraph_.size());
    a.second->computeIncrementalSingleAgentPath();
  }
  replanCBS();
  initialized_ = true;
}

void CBSMPDStarLiteST::timerCallback(const ros::TimerEvent &)
{
  if (!initialized_)
  {
    ROS_INFO("In Timer! Not Initialized Yet!");
    return;
  }
  ROS_INFO("In Timer!");
}

std::vector<std::shared_ptr<Agent>> CBSMPDStarLiteST::getAgents()
{
  std::vector<std::shared_ptr<Agent>> agents;
  for (auto a : agents_)
  {
    agents.push_back(a.second);
  }
  return agents;
}

bool CBSMPDStarLiteST::shouldResample(unsigned int N)
{
  double p = 1.0 - pow(X_, alpha_ * N / S_);
  return ((double)rand()/(double)RAND_MAX) < p;
}

void CBSMPDStarLiteST::printPaths(std::shared_ptr<CTNode> node)
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

void CBSMPDStarLiteST::printConstraints(std::vector<Constraint> constraints)
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

void CBSMPDStarLiteST::printCollision(Collision c)
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


bool CBSMPDStarLiteST::replanCBS()
{
  auto start = std::chrono::high_resolution_clock::now();
  unsigned int node_id = 0;
  unsigned int N = 0;
  std::shared_ptr<CTNode> root = std::make_shared<CTNode>(++node_id, agents_, mamp_helper_);
  for (auto a : agents_)
  {
    root->getPaths().insert({a.first, a.second->getDiscretizedPath()});
  }

  root->detectCollisions();
  root->computeCost();
  open_list_.insert(root->getComparisonTuple(), std::make_tuple(root->getId()), root);
  int iteration=0;
  double avg = 0;
  unsigned int count = 0;
  while (open_list_.size() > 0)
  {
    auto end = std::chrono::high_resolution_clock::now();
    std::chrono::duration<double> diff = end - start;
    if (diff.count() > 1000)
    {
      ROS_INFO("Failed to find path");
      return false;
    }

    iteration++;
    ROS_ERROR("~~~~~~~ CBS Iteration %d ~~~~~~~", iteration);
    // ROS_WARN("Considering resample");
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
        a[i]->computeIncrementalSingleAgentPath();
      }
      open_list_.clear();
      // iteration = 0;
      N = 0;
      node_id = 0;
      root = std::make_shared<CTNode>(++node_id, agents_, mamp_helper_);
      for (auto a : agents_)
      {
        root->getPaths().insert({a.first, a.second->getDiscretizedPath()});
      }
      root->detectCollisions();
      root->computeCost();
      open_list_.insert(root->getComparisonTuple(), std::make_tuple(root->getId()), root);
      ++S_;
    }
    std::shared_ptr<CTNode> node = std::get<2>(open_list_.pop());
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
      ROS_INFO("Average Solve Time: %f", avg);
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
    printCollision(c);
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
      auto start_solve = std::chrono::high_resolution_clock::now();
      succ[i] = new_nodes[i]->getAgents().find(constraints[i].agent_id)->second->computeIncrementalSingleAgentPath(
      MAMP_Helper::getConstraintsForAgent(new_nodes[i]->getConstraints(), constraints[i].agent_id), new_nodes[i]->getMaxConstraintTime(), std::vector<Constraint> {constraints[i]});
      auto end_solve = std::chrono::high_resolution_clock::now();
      std::chrono::duration<double> diff_solve = end_solve - start_solve;
      avg = ((avg * count) + diff_solve.count()) / (count + 1);
      ++count;
      if (succ[i])
      {
        new_nodes[i]->getPaths().erase(constraints[i].agent_id);
        new_nodes[i]->getPaths().insert({constraints[i].agent_id, new_nodes[i]->getAgents().find(constraints[i].agent_id)->second->getDiscretizedPath()});
        new_nodes[i]->detectCollisions();
        new_nodes[i]->computeCost();
        ROS_INFO("Number of collisions: %ld", new_nodes[i]->numCollisions());
        ROS_INFO("Cost: %f", new_nodes[i]->getCost());
        ROS_INFO("Node Id: %d", new_nodes[i]->getId());
        // printCollision(new_nodes[i]->getNextCollision());
        open_list_.insert(new_nodes[i]->getComparisonTuple(), std::make_tuple(new_nodes[i]->getId()), new_nodes[i]);
      }
    }
  }
  // No solution!!!
  return false;
}