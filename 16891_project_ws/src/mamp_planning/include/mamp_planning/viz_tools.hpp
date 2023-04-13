#pragma once

#include <moveit/planning_scene/planning_scene.h>
#include <robot_state_publisher/robot_state_publisher.h>
#include "mamp_planning/vertex.hpp"
#include "mamp_planning/agent.hpp"


// class Vertex;
class VizTools
{
public:
    VizTools(std::shared_ptr<planning_scene::PlanningScene> planning_scene);
    // void run_simulation_single_agent(std::shared_ptr<planning_scene::PlanningScene> planning_scene, const std::vector<std::shared_ptr<Vertex>> &path);
    void run_simulation_single_agent(std::shared_ptr<Agent> agent, int show_path_option=0, int display_rate=10);

    // void run_simulation_all_agents(const std::vector<std::shared_ptr<Vertex>> &path);
    void run_simulation_all_agents(std::vector<std::shared_ptr<Agent>> agents, int show_path_option=0, int display_rate=10);
    

    // run_simulation 
    // void VizTools::run_simulation(std::shared_ptr<planning_scene::PlanningScene> planning_scene, const std::vector<Vertex> vertex_path)


    // // CONSTRUCTOR
    // MAMP_Helper(const std::string &full_world_description); 
    
    // std::shared_ptr<planning_scene::PlanningScene> const &getPlanningScene();// {return planning_scene_;}
    // void setPlanningScene(std::shared_ptr<planning_scene::PlanningScene> new_scene) {planning_scene_ = new_scene;}


    // Edge(std::shared_ptr<Vertex> v1, std::shared_ptr<Vertex> v2);
    // bool const& isValid();
    // void changeValidity(bool validity);
    // std::shared_ptr<Vertex> getOpposingVertex(std::shared_ptr<Vertex> v);
    // double const &getCost();
    // void setTraversalTime(double t);
    // double getTraversalTime();
    // void setDivisions(double divisions);
    // double getDivisions();
    // std::shared_ptr<std::vector<std::vector<double>>> getVertexPositions();
    // std::shared_ptr<std::vector<std::vector<double>>> getVertexPositionsInGivenOrder(std::shared_ptr<Vertex> v1, std::shared_ptr<Vertex> v2);
    // std::pair<double, std::vector<double>> getMagnitudeAndUnitVector(std::shared_ptr<Vertex> start_vertex);

private:
    std::shared_ptr<planning_scene::PlanningScene> planning_scene_;
    ros::Publisher planning_scene_diff_publisher_;
    // double cost_;
    // bool valid_;
    // double traversal_time_;
    // double divisions_;
    
    // std::vector<std::shared_ptr<Vertex>> ordered_vertices_;
    // std::unordered_map<unsigned int, std::shared_ptr<Vertex>> vertices_;
};