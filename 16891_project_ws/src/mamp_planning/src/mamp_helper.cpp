#include "mamp_planning/mamp_helper.hpp"


MAMP_Helper::MAMP_Helper(const std::string &full_world_description)
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
}


// std::shared_ptr<planning_scene::PlanningScene> const &MAMP_Helper::getPlanningScene()
// {
//   return planning_scene_;
// }


bool MAMP_Helper::detectVertexCollision(std::shared_ptr<planning_scene::PlanningScene> planning_scene, 
                                        std::shared_ptr<Vertex> vertex, 
                                        std::shared_ptr<std::vector<std::pair<std::string, std::string>>> list_of_collisions = NULL)
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
            list_of_collisions->push_back(std::pair<std::string, std::string>(it->first.first, it->first.second));
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

// Helper function for detectAgentAgentCollisions
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
std::vector<Collision> MAMP_Helper::detectAgentAgentCollisions(std::unordered_map<std::string, std::vector<std::shared_ptr<Vertex>>> &paths)
{
    // Use the global planning scene to step each agent (planning group) 
    // through their respective path (given input). At each timestep,
    // check for collisions and append collisions to the output vector.

    // Let's find the longest path first
    unsigned int longest_path_size = 0;
    std::vector<std::string> robot_names;
    std::vector<std::string> joint_names;
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
        
        for (int j; j < num_joints; ++j)
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

        // Do the collision check with the multi-agent planning scene
        std::shared_ptr<Vertex> check_vertex = std::make_shared<Vertex>(convertVerticesToJoints(curr_vertices), 0); // the id does not matter here
        std::shared_ptr<std::vector<std::pair<std::string, std::string>>> list_of_collisions;
        // bool test_val = detectVertexCollision(getPlanningScene(), check_vertex, list_of_collisions);
        bool test_val = detectVertexCollision(planning_scene_, check_vertex, list_of_collisions);
        
        if (test_val == true){
            // Go through and create collision objects 

            for (auto collision_pair : *list_of_collisions)
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


// Use this function in PRM to detect whether an edge is valid; it will discretize the edge and check for collisions with the environment
std::pair<bool, std::shared_ptr<Vertex>> MAMP_Helper::detectEdgeCollision(std::shared_ptr<planning_scene::PlanningScene> planning_scene,
                                                                                 std::shared_ptr<Edge> edge,
                                                                                 std::vector<double> &jnt_vel_lim,
                                                                                 double timestep)
{

    // Discretize edge into a series of vertices for collision checking
    std::vector<std::shared_ptr<Vertex>> discrete_steps = discretizeEdge(edge, jnt_vel_lim, timestep);


    // Check each vertex for collisions in the given planning_scene
    // Return false if no collisions are found, with the opposite vertex of the edge
    // Return true if a colliision is found, with the last vertex that was collision free, in the direction of v2.
    bool test_val = false;
    std::shared_ptr<Vertex> last_vertex = discrete_steps[0];

    for (std::shared_ptr<Vertex> discrete_vertex : discrete_steps)
    {
        test_val = detectVertexCollision(planning_scene, discrete_vertex);

        if (test_val == true)
        {
            last_vertex->setId(discrete_steps[-1]->getId());
            return std::pair<bool, std::shared_ptr<Vertex>>(true, last_vertex);
        }
        else
        {
            last_vertex = discrete_vertex;
        }
    }

    return std::pair<bool, std::shared_ptr<Vertex>>(false, discrete_steps[-1]); // The vertex returned here should just be ignored.
}


// Use this function to discretize an edge into smaller vertices based on the max velocity limit and timestep
std::vector<std::shared_ptr<Vertex>> MAMP_Helper::discretizeEdge(std::shared_ptr<Edge> edge, std::vector<double> &jnt_vel_lim, double timestep)
{

    // Get positions of the two vertices
    std::shared_ptr<std::vector<std::vector<double>>> vertices = edge->getVertexPositions();
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
        output.push_back(new_vertex);
    }

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
    
    // Base Variables
    int num_joints = jnt_vel_lim.size();
    // std::shared_ptr<std::vector<std::vector<double>>> start_end_positions = edge->getVertexPositionsInGivenOrder(start_vertex);
    std::vector<double> start_joint_positions = start_vertex->getJointPos();
    std::vector<double> end_joint_positions = edge->getOpposingVertex(start_vertex)->getJointPos();


    //*******************************//
    // Build out the movement_vector //
    //*******************************//

    // Just move how much you can, in the direction we're going, at maximum speed
    // The last increment should be just however much is left.

    // Retrive the unit edge of this vector
    std::pair<double, std::vector<double>> mag_and_unit_pair = edge->getMagnitudeAndUnitVector(start_vertex);
    std::vector<double> unit_vector = mag_and_unit_pair.second;

    std::vector<double> movement_vector(num_joints); // Defined as moving full velocity for the given timestep in direction of unit_vector.
    for (int j=0; j < num_joints; ++j)
    {
        // For context, max_movable_distance[j] = jnt_vel_lim[j] * timestep;
        movement_vector[j] = unit_vector[j] * (jnt_vel_lim[j] * timestep);
    }

    //********************************//
    // Build out the path of vertixes //
    //********************************//
    
    int goals_reached_count = 0;
    std::vector<std::shared_ptr<Vertex>> output;
    start_vertex->setPRMEdge(edge);
    output.push_back(start_vertex);

    while (goals_reached_count < num_joints)
    {
        // For each joint, start with the previous vertex's joint positions
        // If distance between that vertex and the goal vertex is smaller than the step to take, just use the last vertex's joint and mark the goal as reached.
        // At the end, if all goals are reached then add on the goal vertex at the end and return.

        // Reset the goals reached counter
        goals_reached_count = 0;

        // Build the intermediate set of joints
        std::vector<double> new_joints(num_joints);

        for (int j=0; j < num_joints; j++)
        {
            if ((end_joint_positions[j] - output[-1]->getJointPos()[j]) < movement_vector[j])
            {
                // don't move the joint if its within range of its goal
                new_joints[j] = output[-1]->getJointPos()[j];
                goals_reached_count += 1;
            }
            else
            {
                // move the joint by the full amount if its not within range of its goal yet
                new_joints[j] = output[-1]->getJointPos()[j] + movement_vector[j];
            }            
        }

        if (goals_reached_count == num_joints)
        {
            // Add on the final vertex, not the new_joints
            auto final_vertex = edge->getOpposingVertex(start_vertex);
            final_vertex -> setPRMEdge(edge);
            output.push_back(final_vertex);

            // And return from the function
            return output;
        }
        else
        {
            // Add a vertex with these new joints
            std::shared_ptr<Vertex> new_vertex = std::make_shared<Vertex>(new_joints, 0);
            new_vertex->setPRMEdge(edge);
            output.push_back(new_vertex);
        }
    }

    ROS_ERROR("Never should get here in discretizeEdgeDirected");

    return std::vector<std::shared_ptr<Vertex>>();

}