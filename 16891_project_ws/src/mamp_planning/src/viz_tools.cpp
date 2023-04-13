#include "mamp_planning/viz_tools.hpp"


VizTools::VizTools(std::shared_ptr<planning_scene::PlanningScene> planning_scene)
{
    planning_scene_ = planning_scene;
    // planning_scene_diff_publisher_ = node_handle.advertise<moveit_msgs::PlanningScene>("planning_scene", 1);
    
}

void VizTools::run_simulation_single_agent(std::shared_ptr<planning_scene::PlanningScene> planning_scene, const std::vector<std::shared_ptr<Vertex>> &vertex_path)
// void VizTools::run_simulation()
{

    //~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
    // Load up the important things 
    //~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
    ROS_INFO("Loading Visualization");
    ros::NodeHandle node_handle;
    ros::Rate loop_rate(10);
    ros::Publisher pub = node_handle.advertise<sensor_msgs::JointState>("joint_states", 5);
    sensor_msgs::JointState new_joints;

    //~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
    // Setup the initial joints and joint names
    //~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
    std::shared_ptr<Vertex> first_vertex = vertex_path[0];
    new_joints.name.push_back("mobile_1_1");
    new_joints.name.push_back("mobile_1_2");
    new_joints.position.push_back(first_vertex->getJointPos()[0]);
    new_joints.position.push_back(first_vertex->getJointPos()[1]);
    pub.publish(new_joints);


    //~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
    // Cycle through remaining joints
    //~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

    ROS_INFO("Length of path is: %ld", vertex_path.size());

    for(int t=1; t<vertex_path.size() & ros::ok(); ++t)
    {
        ROS_INFO("Made it till here");
        new_joints.header.stamp = ros::Time::now();
        new_joints.position[0] = vertex_path[t]->getJointPos()[0];
        new_joints.position[1] = vertex_path[t]->getJointPos()[1];

        pub.publish(new_joints);
        ros::spinOnce();

        loop_rate.sleep();
    }


    ROS_INFO("Completed Visualization");

}


// void VizTools::run_simulation_all_agents(const std::vector<std::shared_ptr<Vertex>> &vertex_path)
// // void VizTools::run_simulation()
// {

//     //~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
//     // Load up the important things 
//     //~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
//     ROS_INFO("Loading Visualization");
//     ros::NodeHandle node_handle;
//     ros::Rate loop_rate(10);
//     ros::Publisher pub = node_handle.advertise<sensor_msgs::JointState>("joint_states", 5);
//     sensor_msgs::JointState new_joints;

//     //~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
//     // Setup the initial joints and joint names
//     //~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
//     std::shared_ptr<Vertex> first_vertex = vertex_path[0];
//     new_joints.name.push_back("mobile_1_1");
//     new_joints.name.push_back("mobile_1_2");
//     new_joints.position.push_back(first_vertex->getJointPos()[0]);
//     new_joints.position.push_back(first_vertex->getJointPos()[1]);
//     pub.publish(new_joints);


//     //~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
//     // Cycle through remaining joints
//     //~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

//     ROS_INFO("Length of path is: %ld", vertex_path.size());

//     for(int t=1; t<vertex_path.size() & ros::ok(); ++t)
//     {
//         ROS_INFO("Made it till here");
//         new_joints.header.stamp = ros::Time::now();
//         new_joints.position[0] = vertex_path[t]->getJointPos()[0];
//         new_joints.position[1] = vertex_path[t]->getJointPos()[1];

//         pub.publish(new_joints);
//         ros::spinOnce();

//         loop_rate.sleep();
//     }


//     ROS_INFO("Completed Visualization");

// }



