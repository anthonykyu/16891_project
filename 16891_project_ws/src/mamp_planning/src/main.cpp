#include "ros/ros.h"
#include "ros/package.h"
#include "mamp_planning/cbs_mp.hpp"
#include "mamp_planning/pbs_dstar_mp.hpp"
#include "mamp_planning/agent.hpp"
#include <iostream>
#include <fstream>
#include <sstream>

std::vector<std::shared_ptr<Agent>> parseAgentFile(ros::NodeHandle &n, std::string file_name, double timestep)
{
  // Load the robot name, start and goal states from a text file
  

  ifstream file(ros::package::getPath("mamp_planning") + "/src/" + file_name);
  if (!file.is_open())
  {
    ROS_ERROR("Could not open file");
    return std::vector<std::shared_ptr<Agent>>();
  }

  std::string line;
  std::vector<std::shared_ptr<Agent>> agents;
  while (getline(file, line))
  {
    std::string robot_name;
    int dof;
    std::vector<double> start_pos;
    std::vector<double> goal_pos;
    std::string robot_description_name;
    std::string robot_description;
    std::string base_link;
    std::string tip_link;

    // create a stringstream of each line
    std::stringstream ss(line);
    getline(ss, robot_name, ',');
    string dof_string;
    getline(ss, dof_string, ',');
    dof = stoi(dof_string);
    for (int i = 0; i < dof; ++i)
    {
      string str_val;
      getline(ss, str_val, ',');
      double val = stod(str_val);
      start_pos.push_back(val);
    }
    for (int i = 0; i < dof; ++i)
    {
      string str_val;
      getline(ss, str_val, ',');
      double val = stod(str_val);
      goal_pos.push_back(val);
    }
    getline(ss, robot_description_name, ',');
    getline(ss, base_link, ',');
    getline(ss, tip_link, ',');
    n.getParam(robot_description_name, robot_description);

    std::shared_ptr<Agent> a = std::make_shared<Agent>(robot_description, robot_description_name, base_link, tip_link,
                                                      robot_name, timestep, start_pos, goal_pos);
    agents.push_back(a);
  }

  file.close();
  return agents;
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "mamp_planning_node");
  srand(0);
  ROS_INFO("Heyyy");
  double timestep = 0.1;
  std::string world_planning_scene = "multi_mobile_robot_description";
  CBSMP planner_(world_planning_scene, timestep);

  std::vector<std::shared_ptr<Agent>> agents = parseAgentFile(planner_.n_, "agents.txt", timestep);
  planner_.initialize(agents);
  // ROS_INFO("Number of agents: %ld", agents.size());
  // ROS_INFO("Agent ID: %s", agents[0]->getID().c_str());
  // ROS_INFO("Agent Start: %f, %f", agents[0]->getStart()->getJointPos()[0], agents[0]->getStart()->getJointPos()[1]);
  // ROS_INFO("Agent DOF: %ld", agents[0]->getJointVelLimit().size());
  // ROS_INFO("Agent Vel Lim: %f, %f", agents[0]->getJointVelLimit()[0], agents[0]->getJointVelLimit()[1]);
  // ROS_INFO("Agent Upper Lim: %f, %f", agents[0]->getUpperJointLimit()[0], agents[0]->getUpperJointLimit()[1]);
  // ROS_INFO("Agent Lower Lim: %f, %f", agents[0]->getLowerJointLimit()[0], agents[0]->getLowerJointLimit()[1]);
  // ROS_INFO("Agent ID: %s", agents[1]->getID().c_str());
  // ROS_INFO("Agent Start: %f, %f", agents[1]->getStart()->getJointPos()[0], agents[0]->getStart()->getJointPos()[1]);
  // ROS_INFO("Agent DOF: %ld", agents[1]->getJointVelLimit().size());
  // ROS_INFO("Agent Vel Lim: %f, %f", agents[1]->getJointVelLimit()[0], agents[0]->getJointVelLimit()[1]);
  // ROS_INFO("Agent Upper Lim: %f, %f", agents[1]->getUpperJointLimit()[0], agents[0]->getUpperJointLimit()[1]);
  // ROS_INFO("Agent Lower Lim: %f, %f", agents[1]->getLowerJointLimit()[0], agents[0]->getLowerJointLimit()[1]);


  // // Initialize a planning scene
  // // robot_model_loader::RobotModelLoader robot_model_loader("multi_mobile_robot_description");
  // robot_model_loader::RobotModelLoader robot_model_loader("single_mobile_robot_description");
  // // robot_model_loader::RobotModelLoader robot_model_loader("environment_description");
  // // robot_move_loader::RobotModelLoader robo_model_loader("mobile_robot_description");
  // const moveit::core::RobotModelPtr &kinematic_model = robot_model_loader.getModel();
  // auto planning_scene = std::make_shared<planning_scene::PlanningScene>(kinematic_model);

  // // Initialize the agent
  // double ts = 0.1;
  // std::vector<double> jnt_vel_lim{1.0, 1.0};
  // std::vector<double> jnt_lower_lim{0, 0};
  // std::vector<double> jnt_upper_lim{10, 10};
  // auto start_vertex = std::make_shared<Vertex> (std::vector<double>{0.75, 0.75}, 1);
  // auto end_vertex = std::make_shared<Vertex> (std::vector<double>{9.25, 9.25}, 0);

  // auto myPRM = PRM(planning_scene, ts, jnt_vel_lim, jnt_upper_lim, jnt_lower_lim, start_vertex, end_vertex);
  // auto myAStar = AStar(ts);

  // ROS_INFO("Before the PRM building");

  // // PRM.BuildPRM();
  // myPRM.buildPRM();

  // ROS_INFO("We built PRM....YAYYYYYY! AStar now.");

  // std::unordered_map<std::shared_ptr<Edge>, Constraint> constraints;
  // bool succ = myAStar.computePRMPath(start_vertex, end_vertex, constraints);
  // std::vector<std::shared_ptr<Vertex>> prm_path;
  // std::vector<std::shared_ptr<Vertex>> discretized_path;

  // if (succ)
  // {
  //   prm_path = myAStar.getPRMPath(start_vertex, end_vertex);
  //   // ROS_INFO("Starting to discretize");
  //   for (int i = 0; i < prm_path.size() - 1; ++i)
  //   {
  //     discretized_path.push_back(prm_path[i]);
  //     // ROS_INFO("Starting to discretize in da for loop");
  //     if (prm_path[i]->getId() != prm_path[i + 1]->getId())
  //     {
  //       std::vector<std::shared_ptr<Vertex>> dv = MAMP_Helper::discretizeEdgeDirected(prm_path[i], prm_path[i]->getEdges().find(prm_path[i + 1])->second, jnt_vel_lim, ts);
  //       // ROS_INFO("Done with discretize");
  //       std::reverse(dv.begin(), dv.end());
  //       for (auto v : dv)
  //       {
  //         discretized_path.push_back(v);
  //       }
  //     }
  //   }
  //   discretized_path.push_back(prm_path[prm_path.size() - 1]);
  // }
  // ROS_INFO("We here, double YAYYYYYY: %d", succ);

  // for (auto v : prm_path)
  // {
  //   for (int i = 0; i < v->getJointPos().size(); ++i)
  //   {
  //     std::cout << v->getJointPos()[i] << ", ";
  //   }
  //   std::cout << std::endl;
  // }
  // std::cout << prm_path.size() << std::endl;

  // ROS_INFO("Discretized Path below:");

  // for (auto v : discretized_path)
  // {
  //   for (int i = 0; i < v->getJointPos().size(); ++i)
  //   {
  //     std::cout << v->getJointPos()[i] << ", ";
  //   }
  //   std::cout << std::endl;
  // }
  // std::cout << discretized_path.size() << std::endl;
  // ROS_INFO("PRM Size: %ld", myPRM.PRMgraph_.size());
  

  ros::spin();
  return 0;
}
  // auto robot_model_loader = std::make_shared<robot_model_loader::RobotModelLoader>(full_world_description);
  // auto kinematic_model = std::make_shared<moveit::core::RobotModelPtr>(robot_model_loader_->getModel());
  // planning_scene = std::make_shared<planning_scene::PlanningScene>(*kinematic_model_);

  // std::vector<double> joint_values = { 0.0, 0.0, 0.0, -2.9, 0.0, 1.4, 0.0 };
  // const moveit::core::JointModelGroup* joint_model_group = current_state.getJointModelGroup("panda_arm");
  // current_state.setJointGroupPositions(joint_model_group, joint_values);
  // ROS_INFO_STREAM("Test 4: Current state is "
  // << (current_state.satisfiesBounds(joint_model_group) ? "valid" : "not valid"));

// // Let's make an agent and see if the collision testing works
// // Agent test_agent_("../../panda_multiple_arms/robot_description/world_single_mobile.urdf",)

// // Initialize a planning scene
// robot_model_loader::RobotModelLoader robot_model_loader("multi_mobile_robot_description");
// // robot_model_loader::RobotModelLoader robot_model_loader("single_mobile_robot_description");
// // robot_model_loader::RobotModelLoader robot_model_loader("environment_description");
// // robot_move_loader::RobotModelLoader robo_model_loader("mobile_robot_description");
// const moveit::core::RobotModelPtr& kinematic_model = robot_model_loader.getModel();
// planning_scene::PlanningScene planning_scene(kinematic_model);

// collision_detection::CollisionRequest collision_request;
// collision_detection::CollisionResult collision_result;
// planning_scene.checkSelfCollision(collision_request, collision_result);
// ROS_INFO_STREAM("Test 1: Current state is " << (collision_result.collision ? "IN" : "NOT IN") << " self collision");

// ROS_INFO_STREAM("~~~~~~~~~~~~~~~~~~");
// moveit::core::RobotState copied_state = planning_scene.getCurrentStateNonConst();
// ROS_INFO_STREAM("Variables names are " << copied_state.getVariableCount());

// auto names = copied_state.getVariableNames();
// for (auto name : names)
// {
//   ROS_INFO_STREAM("Variable: " << name);
// }

// // ROS_INFO_STREAM("Variables names are " << copied_state.getVariableCount());
// // current_state.setToRandomPositions();

// // Set the variables using the appropriate names
// std::vector<std::string> robot_names {"mobile_1", "mobile_2", "mobile_3", "mobile_4"};
// std::vector<std::string> joint_names;

// for (std::string name : robot_names)
// {
//   joint_names.push_back(name + "_" + "1");
//   joint_names.push_back(name + "_" + "2");
// }

// std::vector<double> positions{1.76, 3, 1.76, 3, 8, 8, 1.76, 3};
// // copied_state.setVariablePositions(positions);
// copied_state.setVariablePositions(joint_names, positions);

// collision_result.clear();
// // collision_detection::AllowedCollisionMatrix acm = planning_scene.getAllowedCollisionMatrix();
// collision_request.contacts = true;
// collision_request.max_contacts = 1000;

// // planning_scene.checkCollision(collision_request, collision_result, current_state, acm);
// planning_scene.checkSelfCollision(collision_request, collision_result, copied_state);
// ROS_INFO_STREAM("Test 2: Current state is " << (collision_result.collision ? "IN" : "NOT IN") << " self collision");

// collision_detection::CollisionResult::ContactMap::const_iterator it;
// for (it = collision_result.contacts.begin(); it != collision_result.contacts.end(); ++it)
// {
//   ROS_INFO("Contact between: %s and %s", it->first.first.c_str(), it->first.second.c_str());
// }