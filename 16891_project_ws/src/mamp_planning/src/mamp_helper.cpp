#include "mamp_planning/mamp_helper.hpp"


MAMP_Helper::MAMP_Helper(const std::string &full_world_description, double timestep)
// Take in the parameter describing the whole world
// Load up the planning scene for the entire world
{
    // robot_model_loader::RobotModelLoader robot_model_loader(full_world_description);
    // const moveit::core::RobotModelPtr& kinematic_model = robot_model_loader.getModel();
    // planning_scene_ = std::make_shared<planning_scene::PlanningScene>(kinematic_model);
    // setPlanningScene(std::make_shared<planning_scene::PlanningScene>(kinematic_model));
    

    auto robot_model_loader_ = std::make_shared<robot_model_loader::RobotModelLoader>(full_world_description);
    auto kinematic_model_ = std::make_shared<moveit::core::RobotModelPtr>(robot_model_loader_->getModel());
    planning_scene_ = std::make_shared<planning_scene::PlanningScene>(*kinematic_model_);
    timestep_ = timestep;
}


std::shared_ptr<planning_scene::PlanningScene> const &MAMP_Helper::getPlanningScene()
{
  return planning_scene_;
}


bool MAMP_Helper::detectVertexCollision(std::shared_ptr<planning_scene::PlanningScene> planning_scene, 
                                        std::shared_ptr<Vertex> vertex, 
                                        std::shared_ptr<std::vector<std::pair<std::string, std::string>>> list_of_collisions,
                                        bool compute_contacts)
{
    // Set the planning scene
    // ROS_INFO("Start detectVertexCollision function");
    moveit::core::RobotState copied_state = planning_scene->getCurrentStateNonConst();
    // ROS_INFO("Before joint pos");
    copied_state.setVariablePositions(vertex->getJointPos());
    // ROS_INFO("%ld", copied_state.getVariableCount());
    // ROS_INFO("size %ld", vertex->getJointPos().size());
    // auto names = copied_state.getVariableNames();
    // ROS_INFO("Printing variable names:");
    // for (auto n : names)
    // {
    //     ROS_INFO("%s", n.c_str());
    // }
    // ROS_INFO("After joint pos set");

    // Prepare to make a request for collision check
    collision_detection::CollisionRequest collision_request;
    // ROS_INFO("Made a request for collision check");

    if (compute_contacts)
    {
        collision_request.contacts = true; // We want to know where the contact happens
        collision_request.max_contacts = 100;
    }
    else
    {
        collision_request.contacts = false; // We want to know where the contact happens
    }
    
    // Check for collisions
    collision_detection::CollisionResult collision_result;
    // ROS_INFO("Set collision request and result");
    // for (int i=0; i<vertex->getJointPos().size(); ++i){
    //     ROS_INFO("Joint %f", copied_state.getVariableNames()[i]);
    //     ROS_INFO("From the vertex it should be %f", vertex->getJointPos()[i]);
    // }

    planning_scene->checkSelfCollision(collision_request, collision_result, copied_state);
    // ROS_INFO("Done checking for vertex collisions");
    // If there IS a collision...
    if (collision_result.collision){
        if (!compute_contacts)
            return true;
        collision_detection::CollisionResult::ContactMap::const_iterator it;
        for (it = collision_result.contacts.begin(); it != collision_result.contacts.end(); ++it)
        {
            // ROS_INFO("Contact between: %s and %s", it->first.first.c_str(), it->first.second.c_str());

            // std::pair<std::string, std::string> 
            if (list_of_collisions)
            {
                // ROS_INFO("Add to list of collisions");
                list_of_collisions->push_back(std::pair<std::string, std::string>(it->first.first, it->first.second));
                // ROS_INFO("Added to list of collisions");
            }
        }
        return true;
    } 
    
    // If there IS NO collision...
    else {
        // ROS_INFO("End detectVertexCollision function");

        // ROS_INFO("No Collisions!");
        return false;
    }
    // ROS_INFO("End detectVertexCollision function");    
    return true; // Should never get here!
}

// Helper function for detectAgentAgentCollisions
std::vector<double> MAMP_Helper::convertVerticesToJoints(std::shared_ptr<planning_scene::PlanningScene> planning_scene, std::vector<std::string> &robot_names, std::vector<std::shared_ptr<Vertex>>& curr_vertices)
{
    moveit::core::RobotState copied_state = planning_scene->getCurrentStateNonConst();
    auto names = copied_state.getVariableNames();
    // ROS_INFO("Number of names: %ld", names.size());
    // ROS_INFO("Printing variable names:");
    // for (auto n : names)
    // {
    //     ROS_INFO("%s", n.c_str());
    // }
    int dof;
    std::vector<double> output_joints;
    for (int i = 0; i < names.size(); i+=dof)
    {
        int idx = 0;
        for (int j = 0; j < robot_names.size(); ++j)
        {
            if (names[i].find(robot_names[j]) != std::string::npos)
            {
                idx = j;
            }
        }
        std::vector<double> curr_joints = curr_vertices[idx]->getJointPos();
        dof = curr_joints.size();
        // ROS_INFO("curr_joints size %ld", curr_joints.size());

        // Add the collected joints into the joint_positions
        for (int k=0; k < dof; ++k)
        {
            output_joints.push_back(curr_joints.at(k));
        }
    }

    return output_joints;
}


// This function is used in the CT node to detect agent-agent collisions
// The input is an unordered map, with the key being the agent id, and the value is the agent discretized path
// static std::vector<Collision> MAMP_Helper::detectAgentAgentCollisions(std::unordered_map<unsigned int, std::vector<std::shared_ptr<Vertex>>> &paths)
std::vector<Collision> MAMP_Helper::detectAgentAgentCollisions(std::unordered_map<std::string, std::vector<std::shared_ptr<Vertex>>> &paths, size_t &num_collisions)
{
    // Use the global planning scene to step each agent (planning group) 
    // through their respective path (given input). At each timestep,
    // check for collisions and append collisions to the output vector.

    // Let's find the longest path first
    unsigned int longest_path_size = 0;
    std::vector<std::string> robot_names;
    std::vector<std::string> joint_names;
    // ROS_INFO("Initialized vectors in agent agent col");
    // int total_joints = 0;
    for (auto path_pair : paths)
    {
        // ROS_INFO("Inside the for loop");
        // Update our longest path size
        if (path_pair.second.size() > longest_path_size)
        {
            longest_path_size = path_pair.second.size();
        }

        // Update our tracking of joints
        int num_joints = path_pair.second.at(0)->getJointPos().size();
        // total_joints = total_joints + num_joints;
        

        for (int j=0; j < num_joints; ++j)
        {
            // ROS_INFO("The number of joints is %ld", num_joints);
            // ROS_INFO("Inside the next for loop");
            joint_names.push_back(path_pair.first + "_" + std::to_string(j+1));
        }
        
        // Update our collection of robot names
        // ROS_INFO("%s", path_pair.first.c_str());
        robot_names.push_back(path_pair.first);
    }


    // Let's cycle through that many timesteps on each path, and feed those into planning scene accordingly.
    // We use the MAMP_Helper's own planning scene for this check
    std::vector<Collision> collisions;
    bool collision_detected = false;
    num_collisions = 0;
    for (int t = 0; t < longest_path_size; ++t)
    {
        // ROS_INFO("Inside 2nd for loop");

        // std::vector<double> joint_positions(joint_names.size());
        // std::vector<double> joint_positions;
        std::vector<std::shared_ptr<Vertex>> curr_vertices;
        
        // Collect the appropriate vertices
        for (auto robot : paths)
        {
            // For this robot, get the appropriate joints for the robot at timestep t
            // ROS_INFO("Inside inner for loop of 2nd for loop");
            
            std::shared_ptr<Vertex> curr_vertex;
            if (t >= robot.second.size())
            {
                // Use the last position of the robot
                // curr_joints = robot.second.at(robot.second.end());
                // curr_joints = robot.second[robot.second.size() - 1]->getJointPos();
                curr_vertex = robot.second[robot.second.size() - 1];
            }
            else
            {
                // curr_joints = robot.second[t]->getJointPos();
                curr_vertex = robot.second[t];
            }
            
            curr_vertices.push_back(curr_vertex);
        }

        // ROS_INFO("About to start the collision check");

        // Do the collision check with the multi-agent planning scene
        std::shared_ptr<Vertex> check_vertex = std::make_shared<Vertex>(convertVerticesToJoints(planning_scene_, robot_names, curr_vertices), 0); // the id does not matter here
        // ROS_INFO("About to start the collision check + 1");
        std::shared_ptr<std::vector<std::pair<std::string, std::string>>> list_of_collisions = std::make_shared<std::vector<std::pair<std::string, std::string>>>();
        // ROS_INFO("About to start the collision check + 2");
        // bool test_val = detectVertexCollision(getPlanningScene(), check_vertex, list_of_collisions);
        bool test_val = detectVertexCollision(planning_scene_, check_vertex, list_of_collisions, !collision_detected);
        
        
        // ROS_INFO("Finished the collision check");
        
        std::string mobile_string= "mob";
        std::string arm_string = "arm";
        
        if (test_val == true){
            collision_detected = true;
            ++num_collisions;
            // Go through and create collision objects 
            for (auto collision_pair : *list_of_collisions)
            {

                // Check if either part of the collision is not an arm or a mobile robot
                if (!((std::strcmp(collision_pair.first.substr(0,3).c_str(), mobile_string.c_str()) != 0 ||
                    std::strcmp(collision_pair.second.substr(0,3).c_str(), mobile_string.c_str()) != 0) ||
                    (std::strcmp(collision_pair.first.substr(0,3).c_str(), arm_string.c_str()) != 0 ||
                    std::strcmp(collision_pair.second.substr(0,3).c_str(), arm_string.c_str()) != 0)))
                {
                    ROS_ERROR("Collision found with a non-agent (i.e. environment) which should not happen if low-level search worked correctly.");
                }
                
                // Return the first collision we've found
                Collision first_collision;
                first_collision.timestep = t * timestep_;
                first_collision.agent_id1 = collision_pair.first;
                first_collision.agent_id2 = collision_pair.second;
                
                // Retrive the edge of that vertex at this timestep.
                for (int n=0; n < robot_names.size(); ++n)
                {
                    // if (robot_names[n] == collision_pair.first)
                    if (std::strcmp(robot_names[n].c_str(), collision_pair.first.c_str()) == 0)
                    {
                        first_collision.location1 = curr_vertices[n]->getPRMEdge();
                        if (first_collision.location1 == nullptr)
                        {
                            first_collision.location1_is_vertex = true;
                            first_collision.location1_vertex = curr_vertices[n];
                        }
                    }
                    // else if (robot_names[n] == collision_pair.second)
                    else if (std::strcmp(robot_names[n].c_str(), collision_pair.second.c_str()) == 0)
                    {
                        first_collision.location2 = curr_vertices[n]->getPRMEdge();
                        if (first_collision.location2 == nullptr)
                        {
                            first_collision.location2_is_vertex = true;
                            first_collision.location2_vertex = curr_vertices[n];
                        }
                    }
                }
                // if (!first_collision.location1 || !first_collision.location2)
                // {
                //     ROS_ERROR("NULL PRM EDGE!!!");
                // }

                // TODO: Add some way to check that both location1 and location2 got filled up
                collisions.push_back(first_collision);
                // return std::vector<Collision>{first_collision};
            }

            // ROS_ERROR("detectAgentAgentCollision: Never should get here. It means you had collisions but didn't return a Collision object from this class");
        }

        // If we get here the timestep is clear, onto the next timestep!

    }
    
    // Return an empty collision vector if no collisions were found!
    return collisions;
    // return std::vector<Collision>();
}


// Use this function in PRM to detect whether an edge is valid; it will discretize the edge and check for collisions with the environment
std::pair<bool, std::shared_ptr<Vertex>> MAMP_Helper::detectEdgeCollision(std::shared_ptr<planning_scene::PlanningScene> planning_scene,
                                                                                 std::shared_ptr<Edge> edge,
                                                                                 std::vector<double> &jnt_vel_lim,
                                                                                 double timestep)
{
    // ROS_INFO("Start detectEdgeCollision");
    // Discretize edge into a series of vertices for collision checking
    // ROS_INFO("Edge has: ", )
    std::vector<std::shared_ptr<Vertex>> discrete_steps = discretizeEdge(edge, jnt_vel_lim, timestep);

    // ROS_INFO("Made Discrete Steps");


    // Check each vertex for collisions in the given planning_scene
    // Return false if no collisions are found, with the opposite vertex of the edge
    // Return true if a colliision is found, with the last vertex that was collision free, in the direction of v2.
    bool test_val = false;
    std::shared_ptr<Vertex> last_vertex = discrete_steps[0];

    // ROS_INFO("Made Gathered the last vertex Steps");
    // ROS_INFO("discrete_steps.size():%ld", discrete_steps.size());
    // for (int i = 0; i < discrete_steps.size(); ++i)
    // {
    //     ROS_INFO("Iteration %d:\t Joint 0 Position:%f", i, discrete_steps[i]->getJointPos()[0]);
    // }
    int temp_divisions = 1;

    for (std::shared_ptr<Vertex> discrete_vertex : discrete_steps)
    {
        // ROS_INFO("In edge collision forloop");
        test_val = detectVertexCollision(planning_scene, discrete_vertex);
        // ROS_INFO("test val is %d", test_val);
        // ROS_INFO("Ran detect vertex collision in the for loop");

        if (test_val) // collision
        {
            // if (last_vertex->getId() == discrete_steps[0]->getId())
            // {
            //     ROS_ERROR("q_near has collided?????");
            // }
            // last_vertex->setId(discrete_steps[discrete_steps.size()-1]->getId());
            edge->setDivisions(temp_divisions);
            return std::pair<bool, std::shared_ptr<Vertex>>(true, last_vertex);
        }
        else
        {
            last_vertex = discrete_vertex;
            temp_divisions++;
        }
    }
    // ROS_INFO("Exiting detectEdgeCollision");ROS_INFO("Start detectEdgeCollision");
    // ROS_INFO("End detectEdgeCollision");
    return std::pair<bool, std::shared_ptr<Vertex>>(false, discrete_steps[discrete_steps.size()-1]); // The vertex returned here should just be ignored.
}


// Use this function to discretize an edge into smaller vertices based on the max velocity limit and timestep
std::vector<std::shared_ptr<Vertex>> MAMP_Helper::discretizeEdge(std::shared_ptr<Edge> edge, std::vector<double> &jnt_vel_lim, double timestep)
{

    // ROS_INFO("DiscretizeEdge START");

    // Get positions of the two vertices
    std::shared_ptr<std::vector<std::vector<double>>> vertices = edge->getVertexPositions();
    int num_joints = vertices->at(0).size();

    // Calculate difference between the vertices
    std::vector<double> diff(num_joints, 0.0);
    double largest_division = 1; 
    double curr_division = 0;
    // ROS_INFO("But diff is %ld", diff.size());

    for (int i=0; i<diff.size(); ++i)
    {
        diff[i] = vertices->at(1).at(i) - vertices->at(0).at(i);
        // ROS_INFO("Here");
        // ROS_INFO("diff %f", diff[i]);
        // ROS_INFO("vel_lim %f", jnt_vel_lim[i]);
        curr_division = abs(diff[i]) / (jnt_vel_lim[i] * timestep);
        // ROS_INFO("curr_division %f", curr_division);

        if (curr_division > largest_division)
        {
            
            largest_division = curr_division;
        }
        // ROS_INFO("largest_division %f", largest_division);

    }
    // ROS_INFO("Calculated differences");


    // Generate number of divisions. Use the largest number of divisions
    double divisions = ceil(largest_division);
    edge->setTraversalTime(timestep * divisions);
    edge->setDivisions(divisions);

    // Create a set of new vertices (including start and end) that can be checked for collisions
    std::vector<std::shared_ptr<Vertex>> output;

    // Add the first vertex
    output.push_back(edge->ordered_vertices_[0]);
    

    for (int i=1; i < (int) (divisions); ++i)
    {
        // Build the intermediate set of joints
        std::vector<double> new_joints(num_joints);
        for (int j=0; j < num_joints; j++)
        {
            new_joints[j] = vertices->at(0).at(j) + (i * (diff.at(j) / divisions));
        }

        std::shared_ptr<Vertex> new_vertex (new Vertex(new_joints, 0));
        new_vertex->setPRMEdge(edge);
        output.push_back(new_vertex);
    }

    // Add the last vertex
    output.push_back(edge->ordered_vertices_[1]);

    // output[0]->setPRMEdge(edge);
    // output[output.size()-1]->setPRMEdge(edge);

    // ROS_INFO("DiscretizeEdge END");

    return output;

}


bool MAMP_Helper::validJointPos(std::shared_ptr<Vertex> vertex, std::vector<double> &jnt_upper_lim, std::vector<double> &jnt_lower_lim)
{
    std::vector<double> vertex_joints = vertex->getJointPos();
    for (int i=0; i<jnt_upper_lim.size(); ++i)
    {
        if (vertex_joints[i] < jnt_lower_lim[i] || vertex_joints[i] > jnt_upper_lim[i])
        {
            return false;
        }
    }
    return true;
}





// Use this function to discretize an edge, it WILL RETURN THE START AND END VERTICES FOR THAT EDGE again in the path
std::vector<std::shared_ptr<Vertex>> MAMP_Helper::discretizeEdgeDirected(std::shared_ptr<Vertex> start_vertex,
                                                                         std::shared_ptr<Edge> edge,
                                                                         std::vector<double> &jnt_vel_lim,
                                                                         double timestep)
{
    
    /*
    We have a set of vertices.
    We have a limiting factor at maximum velocity for one joint.
    Maybe all joints don't have to move at maximum velocity, but they should move in a linear interpolated constant vel profile.
    v1 - a1 - a2 - a3 - a4 - a5 - v2
    Return a1 to a5 only
    

    */


    // What we want from discretizeEdge
    // - 
    std::vector<std::shared_ptr<Vertex>> undirected = discretizeEdge(edge, jnt_vel_lim, timestep);
    // ROS_WARN("Discrteized path for this edge is of size %ld", undirected.size());
    if (undirected[0]->getId() != start_vertex->getId())
    {
        std::reverse(undirected.begin(), undirected.end());
    }
    undirected.erase(undirected.begin());
    undirected.erase(undirected.end());
    return undirected;


    // // Base Variables
    // int num_joints = jnt_vel_lim.size();
    // // std::shared_ptr<std::vector<std::vector<double>>> start_end_positions = edge->getVertexPositionsInGivenOrder(start_vertex);
    // std::vector<double> start_joint_positions = start_vertex->getJointPos();
    // std::vector<double> end_joint_positions = edge->getOpposingVertex(start_vertex)->getJointPos();

    // ROS_INFO("In discretizeEdgeDirected");

    // //*******************************//
    // // Build out the movement_vector //
    // //*******************************//

    // // Just move how much you can, in the direction we're going, at maximum speed
    // // The last increment should be just however much is left.

    // // Retrive the unit edge of this vector
    // std::pair<double, std::vector<double>> mag_and_unit_pair = edge->getMagnitudeAndUnitVector(start_vertex);
    // std::vector<double> unit_vector = mag_and_unit_pair.second;

    // ROS_INFO("Calculated magnitude and unit vector");


    // std::vector<double> movement_vector(num_joints); // Defined as moving full velocity for the given timestep in direction of unit_vector.
    // for (int j=0; j < num_joints; ++j)
    // {
    //     // For context, max_movable_distance[j] = jnt_vel_lim[j] * timestep;
    //     movement_vector[j] = unit_vector[j] * (jnt_vel_lim[j] * timestep);
    // }

    // ROS_INFO("Generated the movement vector");

    // //********************************//
    // // Build out the path of vertixes //
    // //********************************//
    
    // int goals_reached_count = 0;
    // std::vector<std::shared_ptr<Vertex>> output;
    // start_vertex->setPRMEdge(edge);
    // output.push_back(start_vertex);

    // ROS_INFO("About to hit the while loop");

    // while (goals_reached_count < num_joints)
    // {
    //     // For each joint, start with the previous vertex's joint positions
    //     // If distance between that vertex and the goal vertex is smaller than the step to take, just use the last vertex's joint and mark the goal as reached.
    //     // At the end, if all goals are reached then add on the goal vertex at the end and return.

    //     // Reset the goals reached counter
    //     goals_reached_count = 0;

    //     // Build the intermediate set of joints
    //     std::vector<double> new_joints(num_joints);

    //     for (int j=0; j < num_joints; j++)
    //     {
    //         if ((end_joint_positions[j] - output[output.size()-1]->getJointPos()[j]) < movement_vector[j])
    //         {
    //             // don't move the joint if its within range of its goal
    //             new_joints[j] = output[output.size()-1]->getJointPos()[j];
    //             goals_reached_count += 1;
    //         }
    //         else
    //         {
    //             // move the joint by the full amount if its not within range of its goal yet
    //             new_joints[j] = output[output.size()-1]->getJointPos()[j] + movement_vector[j];
    //         }            
    //     }

    //     if (goals_reached_count == num_joints)
    //     {
    //         // Add on the final vertex, not the new_joints
    //         auto final_vertex = edge->getOpposingVertex(start_vertex);
    //         final_vertex -> setPRMEdge(edge);
    //         output.push_back(final_vertex);

    //         // And return from the function
    //         return output;
    //     }
    //     else
    //     {
    //         // Add a vertex with these new joints
    //         std::shared_ptr<Vertex> new_vertex = std::make_shared<Vertex>(new_joints, 0);
    //         new_vertex->setPRMEdge(edge);
    //         output.push_back(new_vertex);
    //     }
    // }

    // ROS_ERROR("Never should get here in discretizeEdgeDirected");

    // return std::vector<std::shared_ptr<Vertex>>();

}