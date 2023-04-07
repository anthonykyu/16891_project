#include "mamp_planning/mamp_helper.hpp"






MAMP_Helper::MAMP_Helper(const std::string &full_world_description)
// Take in the parameter describing the whole world
// Load up the planning scene for the entire world
{
    robot_model_loader::RobotModelLoader robot_model_loader(full_world_description);
    const moveit::core::RobotModelPtr& kinematic_model = robot_model_loader.getModel();
    planning_scene_ = planning_scene::PlanningScene(kinematic_model);
}


static bool MAMP_Helper::detectVertexCollision(std::shared_ptr<planning_scene::PlanningScene> planning_scene, 
                                               std::shared_ptr<Vertex> vertex, 
                                               std::vector<std::pair<std::string, std::string>>& list_of_collisions = NULL)
{
    // Set the planning scene
    moveit::core::RobotState copied_state = planning_scene->getCurrentStateNonConst();
    copied_state.setVariablePositions(vertex->getJointPos());

    // Prepare to make a request for collision check
    collision_detection::CollisionRequest collision_request;
    collision_request.contacts = true; // We want to know where the contact happens
    collision_request.max_contacts = 100;

    // Check for collisions
    collision_detection::CollisionResult collision_result;
    planning_scene->checkSelfCollision(collision_request, collision_result, copied_state);
    
    // If there IS a collision...
    if (collision_result.contact_count > 0){
        collision_detection::CollisionResult::ContactMap::const_iterator it;
        for (it = collision_result.contacts.begin(); it != collision_result.contacts.end(); ++it)
        {
            ROS_INFO("Contact between: %s and %s", it->first.first.c_str(), it->first.second.c_str());

            // std::pair<std::string, std::string> 
            list_of_collisions.push_back(std::pair<std::string, std::string>(it->first.first, it->first.second));
        }
        return true;
    } 
    
    // If there IS NO collision...
    else {
        ROS_INFO("No Collisions!");
        return false;
    }
    
    return true; // Should never get here!
}


std::vector<double> convertVerticesToJoints(std::vector<std::shared_ptr<Vertex>>& curr_vertices)
{
    std::vector<double> output_joints;
    for (std::shared_ptr<Vertex> vertex : curr_vertices)
    {
        std::vector<double> curr_joints;
        curr_joints = vertex->getJointPos();

        // Add the collected joints into the joint_positions
        for (int j=0; j < curr_joints.size(); ++j)
        {
            output_joints.push_back(curr_joints.at(j));
        }
    }

    return output_joints;
}


// This function is used in the CT node to detect agent-agent collisions
// The input is an unordered map, with the key being the agent id, and the value is the agent discretized path
// static std::vector<Collision> MAMP_Helper::detectAgentAgentCollisions(std::unordered_map<unsigned int, std::vector<std::shared_ptr<Vertex>>> &paths)
static std::vector<Collision> MAMP_Helper::detectAgentAgentCollisions(std::unordered_map<std::string, std::vector<std::shared_ptr<Vertex>>> &paths)
{
    // Use the global planning scene to step each agent (planning group) 
    // through their respective path (given input). At each timestep,
    // check for collisions and append collisions to the output vector.


    // Let's find the longest path first
    unsigned int longest_path_size = 0;
    std::vector<str::string> robot_names;
    std::vector<str::string> joint_names;
    // int total_joints = 0;
    for (auto path_pair : paths)
    {
        // Update our longest path size
        if (path_pair.second.size() > longest_path_size)
        {
            longest_path_size = path_pair.second.size();
        }

        // Update our tracking of joints
        int num_joints = path_pair.second.at(0)->getJointPos().size();
        // total_joints = total_joints + num_joints;
        
        for (int j; j < num_joints; ++i)
        {
            joint_names.push_back(path_pair.first + "_" + std::to_string(j+1));
        }
        
        // Update our collection of robot names
        robot_names.push_back(path_pair.first);
    }


    // Let's cycle through that many timesteps on each path, and feed those into planning scene accordingly.
    // We use the MAMP_Helper's own planning scene for this check
    for (int t = 0; t < longest_path_size; ++t)
    {
        // std::vector<double> joint_positions(joint_names.size());
        // std::vector<double> joint_positions;
        std::vector<std::shared_ptr<Vertex>> curr_vertices;
        
        // Collect the appropriate vertices
        for (auto robot : paths)
        {
            // For this robot, get the appropriate joints for the robot at timestep t
            std::vector<double> curr_joints;
            if (t >= robot.second.size())
            {
                // Use the last position of the robot
                // curr_joints = robot.second.at(robot.second.end());
                // curr_joints = robot.second[robot.second.size() - 1]->getJointPos();
                curr_vertices = robot.second[robot.second.size() - 1];
            }
            else
            {
                // curr_joints = robot.second[t]->getJointPos();
                curr_vertices = robot.second[t];
            }
            
            
        }

        // Do the collision check with the multi-agent planning scene
        std::shared_ptr<Vertex> check_vertex = std::make_shared<Vertex>(convertVerticesToJoints(curr_vertices), 0); // the id does not matter here
        std::vector<std::pair<std::string, std::string>> list_of_collisions;
        bool test_val = detectVertexCollision(planning_scene_, check_vertex, list_of_collisions);
        
        if (test_val == true){
            // Go through and create collision objects 

            for (auto collision_pair : list_of_collisions)
            {
                // Check if either part of the collision is not an arm or a mobile robot
                if (collision_pair.first.substr(0,3) != "mob" || collision_pair.first.substr(0,3) != "arm")
                {
                    ROS_ERROR("Collision found with a non-agent (i.e. environment) which should not happen if low-level search worked correctly.");
                }
                
                // Return the first collision we've found
                Collision first_collision;
                first_collision.timestep = t;
                first_collision.agent_id1 = collision_pair.first;
                first_collision.agent_id2 = collision_pair.second;
                
                // Retrive the edge of that vertex at this timestep.
                for (int n=0; n < robot_names.size(); ++n)
                {
                    if (robot_names[n] == collision_pair.first)
                    {
                        first_collision.location1 = curr_vertices[n]->getPRMEdge();
                    }
                    else if (robot_names[n] == collision_pair.second)
                    {
                        first_collision.location2 = curr_vertices[n]->getPRMEdge();
                    }
                }

                // TODO: Add some way to check that both location1 and location2 got filled up
                return std::vector<Collision>{first_collision};
            }

            ROS_ERROR("detectAgentAgentCollision: Never should get here. It means you had collisions but didn't return a Collision object from this class");
        }

        // If we get here the timestep is clear, onto the next timestep!

    }
    
    // Return an empty collision vector if no collisions were found!
    return std::vector<Collision>();
}


// TODO: This function should return the last possible vector that it can go till, right?
// Use this function in PRM to detect whether an edge is valid; it will discretize the edge and check for collisions with the environment
// static bool MAMP_Helper::detectEdgeCollision(std::shared_ptr<planning_scene::PlanningScene> planning_scene, std::shared_ptr<Edge> edge,
//                                              std::vector<double> &jnt_vel_lim, double timestep)
static std::pair<bool, std::shared_ptr<Vertex>> MAMP_Helper::detectEdgeCollision(std::shared_ptr<planning_scene::PlanningScene> planning_scene,
                                                                                 std::shared_ptr<Edge> edge,
                                                                                 std::vector<double> &jnt_vel_lim,
                                                                                 double timestep)
{
    // This function will be used in the PRM generation.
    // Essentially this calls the discretizeEdge function to break down the given edge input
    // and then calls detectVertexCollision to check for collisions at the vertex with the environment

    std::vector<std::shared_ptr<Vertex>> discrete_steps = discretizeEdge(edge, jnt_vel_lim, timestep);

    bool test_val = false;
    std::shared_ptr<Vertex> last_vertex = discrete_steps[0];

    for (std::shared_ptr<Vertex> discrete_vertex : discrete_steps)
    {
        test_val = detectVertexCollision(planning_scene, discrete_vertex);

        if (test_val == true)
        {
            return std::pair<bool, std::shared_ptr<Vertex>>(true, last_vertex);
        }
        else{
            last_vertex = discrete_vertex;
        }
    }


    return std::pair<bool, std::shared_ptr<Vertex>>(false, last_vertex);
}


// Use this function to discretize an edge into smaller vertices based on the max velocity limit and timestep
static std::vector<std::shared_ptr<Vertex>> MAMP_Helper::discretizeEdge(std::shared_ptr<Edge> edge, std::vector<double> &jnt_vel_lim, double timestep)
// static std::vector<std::vector<double>> MAMP_Helper::discretizeEdge(std::shared_ptr<Edge> edge, std::vector<double> &jnt_vel_lim, double timestep)
{
    // This function breaks down a given edge into a list of vertices to check for collisions
    // The discretization is based off of the max joint velocity and timestep given as inputs

    // Get positions of the two vertices
    std::shared_ptr<std::vector<std::vector<double>>> vertices = edge->getVertices();
    int num_joints = vertices->at(0).size();

    // Calculate difference between the vertices
    std::vector<double> diff(num_joints);
    double largest_division = 0; 
    double curr_division = 0;
    for (int i=0; i<diff.size(); ++i)
    {
        diff[i] = vertices->at(1).at(i) - vertices->at(0).at(i);
        curr_division = diff[i] / (jnt_vel_lim[i] * timestep);

        if (curr_division > largest_division)
        {
            largest_division = curr_division;
        }
    }

    // Generate number of divisions. Use the largest number of divisions
    double divisions = ceil(largest_division);

    // Create a set of new vertices (including start and end) that can be checked for collisions
    std::vector<std::shared_ptr<Vertex>> output;
    for (int i=0; i < (int) (divisions+1); ++i)
    {
        // Build the intermediate set of joints
        std::vector<double> new_joints(num_joints);
        for (int j=0; j < num_joints; j++)
        {
            new_joints[j] = vertices->at(0).at(j) + (i * (diff.at(j) / divisions));
        }

        std::shared_ptr<Vertex> new_vertex (new Vertex(new_joints, 0));
    }

    return output;



    // double diff_x = vertices[1].first - vertices[0].first;
    // double diff_y = vertices[1].second - vertices[0].second;

    // double max_x_step = jnt_vel_lim[0] * timestep;
    // double max_y_step = jnt_vel_lim[1] * timestep;

    // double x_steps = diff_x / max_x_step;
    // double y_steps = diff_y / max_y_step;

    // int divisions = (int) ceil(max(x_steps, y_steps));
    // double x_step = diff_x/divisions;
    // double y_step = diff_y/divisions;

    // // double curr_x = vertices[0].first;
    // // double curr_y = vertices[0].second;

    // std::vector<std::shared_ptr<Vertex>> output;
    // for (int i=0, i < divisions + 1, ++i)
    // {   
    //     std::shared_ptr<Vertex> new_vertex (new Vertex(std::vector<double> {vertices[0].first + x_step*i, vertices[0].second + y_step*i}, 0)); // Keeping all id's 0 here as they are irrelevant.
    //     output.push_back(new_vertex);
    // }

    // // return std::vector<std::shared_ptr<Vertex>>();
    // return output;



}


static bool MAMP_Helper::validJointPos(std::shared_ptr<Vertex> vertex, std::vector<double> &jnt_upper_lim, std::vector<double> &jnt_lower_lim)
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





// Use this function to discretize an edge
static std::vector<std::shared_ptr<Vertex>> MAMP_Helper::discretizeEdgeDirected(std::shared_ptr<Vertex> start_vertex, std::shared_ptr<Edge> edge, std::vector<double> &jnt_vel_lim, double timestep)
{
    // This function breaks down a given edge into a list of vertices where order matters (starting from start_vertex).
    // This functions the same as discretizeEdge, but will be used when finding a path
    // for an agent. This will be used in A* or D* Lite, and the output are the vertices
    // inserted into the Agent's path.

    // We need each step to take the same amount of time.



    // Make sure to also point back to the edge that the new vertex was from so we can backtrack later.
    return std::vector<std::shared_ptr<Vertex>>();
}