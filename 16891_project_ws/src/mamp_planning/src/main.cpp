#include "ros/ros.h"
#include "mamp_planning/cbs_mp.hpp"
#include "mamp_planning/pbs_dstar_mp.hpp"
#include "mamp_planning/agent.hpp"

int main(int argc, char **argv)
{
  ros::init(argc, argv, "mamp_planning_node");
  ROS_INFO("Heyyy");
  CBSMP planner_;







  // Let's make an agent and see if the collision testing works
  // Agent test_agent_("../../panda_multiple_arms/robot_description/world_single_mobile.urdf",)


  // Initialize a planning scene
  robot_model_loader::RobotModelLoader robot_model_loader("multi_mobile_robot_description");
  // robot_model_loader::RobotModelLoader robot_model_loader("single_mobile_robot_description");
  // robot_model_loader::RobotModelLoader robot_model_loader("environment_description");
  // robot_move_loader::RobotModelLoader robo_model_loader("mobile_robot_description");
  const moveit::core::RobotModelPtr& kinematic_model = robot_model_loader.getModel();
  planning_scene::PlanningScene planning_scene(kinematic_model);


  collision_detection::CollisionRequest collision_request;
  collision_detection::CollisionResult collision_result;
  planning_scene.checkSelfCollision(collision_request, collision_result);
  ROS_INFO_STREAM("Test 1: Current state is " << (collision_result.collision ? "IN" : "NOT IN") << " self collision");


  ROS_INFO_STREAM("~~~~~~~~~~~~~~~~~~");
  moveit::core::RobotState copied_state = planning_scene.getCurrentStateNonConst();
  ROS_INFO_STREAM("Variables names are " << copied_state.getVariableCount());

  auto names = copied_state.getVariableNames();
  for (auto name : names) 
  {
    ROS_INFO_STREAM("Variable: " << name);
  }

  // ROS_INFO_STREAM("Variables names are " << copied_state.getVariableCount());
  // current_state.setToRandomPositions();

  // Set the variables using the appropriate names
  std::vector<std::string> robot_names {"mobile_1", "mobile_2", "mobile_3", "mobile_4"};
  std::vector<std::string> joint_names;

  for (std::string name : robot_names)
  {
    joint_names.push_back(name + "_" + "1");
    joint_names.push_back(name + "_" + "2");
  }


  std::vector<double> positions{1.76, 3, 1.76, 3, 8, 8, 1.76, 3};
  // copied_state.setVariablePositions(positions);
  copied_state.setVariablePositions(joint_names, positions);

  collision_result.clear();
  // collision_detection::AllowedCollisionMatrix acm = planning_scene.getAllowedCollisionMatrix();
  collision_request.contacts = true;
  collision_request.max_contacts = 1000;

  // planning_scene.checkCollision(collision_request, collision_result, current_state, acm);
  planning_scene.checkSelfCollision(collision_request, collision_result, copied_state);
  ROS_INFO_STREAM("Test 2: Current state is " << (collision_result.collision ? "IN" : "NOT IN") << " self collision");

  collision_detection::CollisionResult::ContactMap::const_iterator it;
  for (it = collision_result.contacts.begin(); it != collision_result.contacts.end(); ++it)
  {
    ROS_INFO("Contact between: %s and %s", it->first.first.c_str(), it->first.second.c_str());
  }


  ROS_INFO("We here");
  
  // std::vector<double> joint_values = { 0.0, 0.0, 0.0, -2.9, 0.0, 1.4, 0.0 };
  // const moveit::core::JointModelGroup* joint_model_group = current_state.getJointModelGroup("panda_arm");
  // current_state.setJointGroupPositions(joint_model_group, joint_values);
  // ROS_INFO_STREAM("Test 4: Current state is "
                  // << (current_state.satisfiesBounds(joint_model_group) ? "valid" : "not valid"));



  ros::spin();
  return 0;
}