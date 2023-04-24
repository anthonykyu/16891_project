#include "mamp_planning/viz_tools.hpp"


VizTools::VizTools(std::shared_ptr<planning_scene::PlanningScene> planning_scene)
{
    planning_scene_ = planning_scene;
    // planning_scene_diff_publisher_ = node_handle.advertise<moveit_msgs::PlanningScene>("planning_scene", 1);    
}




void VizTools::run_simulation_single_agent(std::shared_ptr<Agent> agent, int show_path_option, int display_rate)
{
    // show_path_option = 0 for showing the discretized path
    // show_path_option = 1 for showing the prm path 
    // show_path_option = 2 for showing the prm path and then the discretized path

    //~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
    // Load up the important things 
    //~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
    ROS_INFO("Loading Visualization");
    ros::NodeHandle node_handle;
    ros::Rate loop_rate(display_rate);
    ros::Publisher pub = node_handle.advertise<sensor_msgs::JointState>("joint_states", 5);
    std::vector<shared_ptr<Vertex>> vertex_path;
    
    if (show_path_option == 0)
    {
        vertex_path = agent->getDiscretizedPath();
    }
    else if (show_path_option == 1)
    {
        vertex_path = agent->getPRMPath();
    }
    else
    {
        if (show_path_option != 2)
        {
            ROS_ERROR("run_simulation_single_agent: Invalid input value that is not 0, 1, or 2, it is ~%d~. Resorting to value of 2.", show_path_option);
        }

        vertex_path = agent -> getPRMPath();
        auto vertex_path_2 = agent->getDiscretizedPath();

        for (auto vertex : vertex_path_2)
        {
            vertex_path.push_back(vertex);
        }
    }
    

    //~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
    // Setup the initial joints and joint names
    //~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
    sensor_msgs::JointState new_joints;
    std::shared_ptr<Vertex> first_vertex = vertex_path[0];
    int num_joints = first_vertex->getJointPos().size();
    ROS_INFO("Agent's name is: %s", agent->getID().c_str());
    for (int j=0; j< num_joints; ++j)
    {
        new_joints.name.push_back(agent->getID() + "_" + std::to_string(j+1));

        ROS_INFO("We get %s", (agent->getID() + "_" + std::to_string(j+1)).c_str());

        new_joints.position.push_back(first_vertex->getJointPos()[j]);
    }
    new_joints.header.stamp = ros::Time::now();
    pub.publish(new_joints);


    //~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
    // Cycle through remaining joints
    //~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

    ROS_INFO("Length of path is: %ld", vertex_path.size());

    for(int ts=1; ts<vertex_path.size() & ros::ok(); ++ts)
    {
        ROS_INFO("~~~~~~~~~~~ Timestep: %d ~~~~~~~~~~", ts);
        new_joints.header.stamp = ros::Time::now();
        new_joints.position[0] = vertex_path[ts]->getJointPos()[0];
        new_joints.position[1] = vertex_path[ts]->getJointPos()[1];

        pub.publish(new_joints);
        // ros::spinOnce();

        loop_rate.sleep();
    }


    ROS_INFO("Completed Visualization");

}



void VizTools::run_simulation_all_agents(std::vector<std::shared_ptr<Agent>> agents, int show_path_option, int display_rate, double wait_time)
{
    // show_path_option = 0 for showing the discretized path
    // show_path_option = 1 for showing the prm path 
    // show_path_option = 2 for showing the prm path and then the discretized path


    //~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
    // Load up the important things 
    //~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
    ROS_INFO("Loading Visualization for all agents");
    ros::NodeHandle node_handle;
    ros::Rate loop_rate(display_rate);
    ros::Publisher pub = node_handle.advertise<sensor_msgs::JointState>("joint_states", 0);
    std::vector<std::vector<shared_ptr<Vertex>>> all_paths;
    int num_agents = agents.size();

    int longest_path = 0;

    for (int a=0; a<num_agents; ++a)
    {
        auto agent = agents[a];
        std::vector<shared_ptr<Vertex>> agent_path;

        //~~~~~~ Gather the path of that agent subject to the visualization preference we have
        if (show_path_option == 0)
        {
            agent_path = agent->getDiscretizedPath();
        }
        else // (show_path_option == 1)
        {
            if (show_path_option != 1)
            {
                ROS_ERROR("run_simulation_multi_agent: Invalid input value that is not 0 or 1, it is ~%d~. Resorting to value of 1.", show_path_option);
            }
            agent_path = agent->getPRMPath();
        }

        if (agent_path.size() > longest_path)
        {
            longest_path = agent_path.size();
        }

        //~~~~~~ Add the path into our total cumulation of paths
        all_paths.push_back(agent_path);
    }

    
    //~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
    // Setup the initial joints and joint names
    //~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
    sensor_msgs::JointState new_joints;
    new_joints.header.seq = 1;
    for (int a=0; a<num_agents; ++a)
    {
        std::shared_ptr<Vertex> first_vertex = all_paths[a][0];    
        int num_joints = first_vertex->getJointPos().size();
        auto agent = agents[a];
        ROS_INFO("Preparing simulation for Agent: %s", agent->getID().c_str());
        for (int j=0; j< num_joints; ++j)
        {
            new_joints.name.push_back(agent->getID() + "_" + std::to_string(j+1));

            ROS_INFO("Joint %s", (agent->getID() + "_" + std::to_string(j+1)).c_str());

            new_joints.position.push_back(first_vertex->getJointPos()[j]);
        }
    }


    //~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
    // Ensure we hold at the first waypoint for a moment
    //~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

    int pub_first_state = 0;
    while (ros::ok() && pub_first_state < wait_time*display_rate)
    {
        new_joints.header.stamp = ros::Time::now();
        new_joints.header.seq++;
        pub.publish(new_joints);
        loop_rate.sleep();
        pub_first_state ++;

    }


    //~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
    // Cycle through remaining joints
    //~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

    // Cycle through length of longest path (that's the timestep)
    // Go agent by agent
    // Retrieve their location at that timestep
    // If the timestep is longer than their discretized path length, return the last position

    for(int ts=1; ts<longest_path & ros::ok(); ++ts)
    {
        ROS_INFO("~~~~~~~~~~~ Timestep: %d ~~~~~~~~~~", ts);
        
        new_joints.header.stamp = ros::Time::now();
        new_joints.header.seq++;
        
        int joints_idx = 0;

        for (int a=0; a<num_agents; ++a)
        {
            int path_idx_for_agent;
            if (ts >= all_paths[a].size()) // the agent has already reached its goal
            {
                path_idx_for_agent = all_paths[a].size()-1;
            }
            else
            {
                path_idx_for_agent = ts;
            }

            std::vector<double> joints = all_paths[a][path_idx_for_agent]->getJointPos();

            for (int j=0; j<joints.size(); ++j)
            {
                new_joints.position[joints_idx] = joints[j];
                joints_idx++;
            }
        }

        pub.publish(new_joints);
        // ros::spinOnce();


        loop_rate.sleep();
    }

    ROS_INFO("--------------------- Completed Visualization ---------------------");

}

