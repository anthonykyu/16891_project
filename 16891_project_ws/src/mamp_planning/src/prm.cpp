#include "mamp_planning/prm.hpp"

PRM::PRM(std::shared_ptr<planning_scene::PlanningScene> planning_scene, double timestep, 
        std::vector<double> &jnt_vel_lim, std::vector<double> &jnt_upper_lim, std::vector<double> &jnt_lower_lim,
        std::vector<std::shared_ptr<Vertex>> waypoints, std::shared_ptr<collision_detection::AllowedCollisionMatrix> acm)
{
  radius_ = 2;
  num_samples_ = 0;
  expansion_factor_= 0.1;
  connectivity_ = 8;
  waypoints_ = waypoints;
//   start_ = start;
//   goal_ = goal;
  planning_scene_ = planning_scene;
  timestep_ = timestep;
  dof_ = jnt_upper_lim.size();
  node_id_ = waypoints_.size();
  jnt_vel_lim_ = jnt_vel_lim;
  jnt_lower_lim_ = jnt_lower_lim;
  jnt_upper_lim_ = jnt_upper_lim;
  acm_ = acm;
//   ROS_INFO("We have a new PRM!");

}

// bool PRM::checkCollision(vector<double> joint_pos)
// {
//     return false;
// }

double PRM::getDistance(shared_ptr<Vertex> v1, shared_ptr<Vertex> v2)
{
  double distance = 0;
  for (int i = 0; i < v1->getJointPos().size(); ++i)
  {
    double d = v1->getJointPos()[i] - v2->getJointPos()[i];
    distance += d*d;
  }
  return sqrt(distance);
}

void PRM::getNeighbors(shared_ptr<Vertex> q_new, double radius)
{
  for (int i = 0; i < PRMgraph_.size(); ++i)
  {
    // Don't connect with yourself
    if (q_new->getId() == PRMgraph_[i]->getId())
    {
      continue;
    }

    // If the node is within range, then add it to my neighbors
    double distance = getDistance(q_new, PRMgraph_[i]);
    if (distance <= radius)
    {
        q_new->addToNeighborhood(PRMgraph_[i]);
    }
  }
}

shared_ptr<Vertex> PRM::getRandomVertex()
{
    // ROS_INFO("GetRandomVertex starts here");
    // use Vertex class to create a new vertex
    vector<double> q_rand_pos(dof_);
    for (int i = 0; i < dof_; ++i)
    {
      q_rand_pos[i] = jnt_lower_lim_[i] + (((double)rand() / (double)RAND_MAX) * (jnt_upper_lim_[i] - jnt_lower_lim_[i])) ;
    //   printf("This is %f\n", q_rand_pos[i]);
    }
    shared_ptr<Vertex> q_rand = make_shared<Vertex>(q_rand_pos, node_id_++);
    // ROS_INFO("GetRandomVertex dof_ is: %d", dof_);
    // ROS_INFO("GetRandomVertex middle point here");

    while(MAMP_Helper::detectVertexCollision(planning_scene_, acm_, q_rand, nullptr))
    {
        for (int i = 0; i < dof_; ++i)
        {
        q_rand_pos[i] = jnt_lower_lim_[i] + (((double)rand() / (double)RAND_MAX) * (jnt_upper_lim_[i] - jnt_lower_lim_[i])) ;
        // printf("This is %f\n", q_rand_pos[i]);
        }
        q_rand->setJointPos(q_rand_pos);
    }
    // ROS_INFO("GetRandomVertex ends here");
    return q_rand;
}

// shared_ptr<Vertex> PRM::getNewVertex(shared_ptr<Vertex> q_near,shared_ptr<Vertex> q, int r)
// {
//     // create a new vertex
//     double distance = getDistance(q_near, q);
//     int num_steps = (int)(distance / r);

//     if (num_steps < 2)
//     {
//         shared_ptr<Vertex> q_new = make_shared<Vertex>(q->getJointPos(), q->getId());
//         return q_new;
//     }

//     auto unit_vector = q_near->getJointPos();
//     for (int i = 0; i < q->getJointPos().size(); ++i)
//     {
//         unit_vector[i] = (q->getJointPos()[i] - q_near->getJointPos()[i]) / distance;
//         unit_vector[i] = q_near->getJointPos()[i] + unit_vector[i] * epsilon_;

//     }
//     shared_ptr<Vertex> q_new = make_shared<Vertex>(unit_vector, node_id_++);

//     return q_new;

// }

// shared_ptr<Vertex> PRM::getNearestVertex(shared_ptr<Vertex> q, vector<shared_ptr<Vertex>> nodes, int max_id)
// {
//     double min_distance = 1000000;
//     shared_ptr<Vertex> q_near;
//     for(int i = 0; i < nodes.size(); i++)
//     {
//         if (nodes[i]->getComponentId() != max_id)
//         {
//             continue;
//         }

//         double distance = getDistance(q, nodes[i]);
//         if (distance < min_distance)
//         {
//             min_distance = distance;
//             q_near = nodes[i];
//         }
//     }
//     return q_near;
// }

// bool PRM::connect(shared_ptr<Vertex> q1, shared_ptr<Vertex> q2)
// {
//     double distance = getDistance(q1, q2);
//     int num_steps = (int)(distance / epsilon_);
//     if (num_steps < 2)
//     {
//         return true;
//     }

    
//     for (int i = 0; i < num_steps; ++i)
//     {
//         // creat a double vector with the same size as the joint position
//         vector<double> unit_vector;
//         for (int j = 0; j < q1->getJointPos().size(); ++j)
//         {
//             unit_vector[j] = q1->getJointPos()[j] + (q2->getJointPos()[j] - q1->getJointPos()[j]) * i / (num_steps-1);        
//         }

//         // how do I check collision for these joint positionss in the world?        
//         if (!checkCollision(unit_vector)) 
//         {
//             return false;
//         }
//     }
//     return false;
// }

// void PRM::getPath(vector<shared_ptr<Vertex>> nodes)
// {
//     shared_ptr<Vertex> q = goal_;
//     while (q->getId() != start_->getId())
//     {
//         PRMpath_.push_back(q);
//         q = q->getParent();
//     }
//     PRMpath_.push_back(start_);
// }

void PRM::expandPRM()
{
    int i = 0;
    for (; i < (expansion_factor_ * num_samples_); ++i)
    {
        // ROS_INFO("Component value: %d", component_);
        // ROS_INFO("Start and Goal Comp ID: %d, %d", start_->getComponentId(), goal_->getComponentId());
        shared_ptr<Vertex> q_rand = getRandomVertex();
        //NOTE: if the given configuration is valid, need to check with Hardik
        // if (CheckCollision(q_rand->getJointPos()))
        // {
        //     continue;
        // }

        // ROS_INFO("~~~~ Iteration %d", i);

        PRMgraph_.push_back(q_rand);
        q_rand->setComponentId(component_++);
        getNeighbors(q_rand, radius_);

        // ROS_INFO("We have NEIGHBOURS that we know.");

        // if the neighborhood is empty, then the new vertex is a new component_
        if (q_rand->getNeighborhood().empty())
        {

            q_rand->setComponentId(component_);
            // ROS_INFO("q_rand comp id %d", q_rand->getComponentId());
            component_++;
            continue;
        }
        
        // ROS_INFO("It has neighbors. Let's add edges.");

        for(int j = 0; j < q_rand->getNeighborhood().size(); j++)
        {
            shared_ptr<Vertex> q_near = q_rand->getNeighborhood()[j];
            // ROS_INFO("Obtained a neighbor");

            // If you're already in the same component_, you can skip everything else!
            if (q_rand->getComponentId() == q_near->getComponentId() && q_near->getEdges().size() > connectivity_)
            {
                continue;
            }
            // ROS_INFO("Got past componnent check");


            shared_ptr<Edge> edge = make_shared<Edge>(q_near, q_rand);
            // ROS_INFO("Q_near has joints: %ld", q_near->getJointPos().size());
            auto coll = MAMP_Helper::detectEdgeCollision(planning_scene_, acm_, edge, jnt_vel_lim_, timestep_);


            // ROS_INFO("Executed Collision Check");
            if (coll.first == false) // no collision
            {
                q_rand->addEdge(q_near, edge);
                q_near->addEdge(q_rand, edge);
                // PRMgraph_.push_back(q_rand);
                // ROS_INFO("Traversal Time for normal edge: %f \t %f", edge->getDivisions(), edge->getTraversalTime());
            }

            // otherwise add an edge until the point it is collision free
            else
            {
                if (q_near->getId() != coll.second->getId())
                {
                    // coll.second->setId(q_rand->getId());
                    // coll.second->setComponentId(q_rand->getComponentId());
                    coll.second->setId(node_id_++);
                    coll.second->setComponentId(q_near->getComponentId());
                    // ROS_INFO("q_near comp id: %d", q_near->getComponentId());
                    // ROS_INFO("coll.second comp id: %d", coll.second->getComponentId());

                    PRMgraph_.push_back(coll.second);

                    shared_ptr<Edge> collision_free_edge = make_shared<Edge>(q_near, coll.second);
                    // ROS_INFO("edge divisions %f", edge.getDivisions());

                    collision_free_edge->setDivisions(edge->getDivisions());
                    collision_free_edge->setTraversalTime(collision_free_edge->getDivisions() * timestep_);
                    // ROS_INFO("Traversal Time for REWRITTEN edge: %f \t %f", collision_free_edge->getDivisions(), collision_free_edge->getTraversalTime());
                    coll.second->addEdge(q_near, collision_free_edge);
                    q_near->addEdge(coll.second, collision_free_edge);

                }

                // No else needed because you don't need an edge to yourself.
                continue;
            }

            if(q_rand->getComponentId() != q_near->getComponentId())
            {
                // merge two components
                // q_rand->setComponentId(q_near->getComponentId());

                int overwrite_id = q_near->getComponentId();
                // ROS_INFO("overwrite id %d", overwrite_id);
                int new_id = q_rand->getComponentId();
                // ROS_INFO("new_id id %d", new_id);

                for (auto v : PRMgraph_)
                {
                    if (v->getComponentId() == overwrite_id)
                    {
                        v->setComponentId(new_id);
                    }
                }
            }
            // Have a neighbourhood, but can't connect to anyone
            // else
            // {
            //     q_rand->setComponentId(component_);
            //     component_++;
            // } 
        }
        
    }
    num_samples_ += i;
}

void PRM::buildPRM()
{
    component_ = 0;
    for (int i = 0; i < waypoints_.size(); ++i)
    {
        waypoints_[i]->setComponentId(component_++);
        PRMgraph_.push_back(waypoints_[i]);
    }
    // start_->setComponentId(component_++);
    // goal_->setComponentId(component_++);
    // PRMgraph_.push_back(start_);
    // PRMgraph_.push_back(goal_);
    // ROS_INFO("Start and Goal Comp ID: %d, %d", start_->getComponentId(), goal_->getComponentId());
    
    ROS_INFO("BuldPRM starts here");
    bool not_connected = true;
    for (int i = 0; not_connected; ++i, ++num_samples_)
    {
        not_connected = false;
        for (int j = 0; j < waypoints_.size()-1; ++j)
        {
            not_connected = not_connected || waypoints_[j]->getComponentId() != waypoints_[j+1]->getComponentId();
        }
        if (!not_connected)
            break;
        // ROS_INFO("Component value: %d", component_);
        // ROS_INFO("Start and Goal Comp ID: %d, %d", start_->getComponentId(), goal_->getComponentId());
        shared_ptr<Vertex> q_rand = getRandomVertex();
        //NOTE: if the given configuration is valid, need to check with Hardik
        // if (CheckCollision(q_rand->getJointPos()))
        // {
        //     continue;
        // }

        ROS_INFO("~~~~ Iteration %d", i);

        PRMgraph_.push_back(q_rand);
        q_rand->setComponentId(component_++);
        getNeighbors(q_rand, radius_);

        // ROS_INFO("We have NEIGHBOURS that we know.");

        // if the neighborhood is empty, then the new vertex is a new component_
        if (q_rand->getNeighborhood().empty())
        {

            q_rand->setComponentId(component_);
            // ROS_INFO("q_rand comp id %d", q_rand->getComponentId());
            component_++;
            continue;
        }
        
        // ROS_INFO("It has neighbors. Let's add edges.");

        for(int j = 0; j < q_rand->getNeighborhood().size(); j++)
        {
            shared_ptr<Vertex> q_near = q_rand->getNeighborhood()[j];
            // ROS_INFO("Obtained a neighbor");

            // If you're already in the same component_, you can skip everything else!
            if (q_rand->getComponentId() == q_near->getComponentId() && q_near->getEdges().size() > connectivity_)
            {
                continue;
            }
            // ROS_INFO("Got past componnent check");


            shared_ptr<Edge> edge = make_shared<Edge>(q_near, q_rand);
            // ROS_INFO("Q_near has joints: %ld", q_near->getJointPos().size());
            auto coll = MAMP_Helper::detectEdgeCollision(planning_scene_, acm_, edge, jnt_vel_lim_, timestep_);


            // ROS_INFO("Executed Collision Check");
            if (coll.first == false) // no collision
            {
                q_rand->addEdge(q_near, edge);
                q_near->addEdge(q_rand, edge);
                // PRMgraph_.push_back(q_rand);
                // ROS_INFO("Traversal Time for normal edge: %f \t %f", edge->getDivisions(), edge->getTraversalTime());
            }

            // otherwise add an edge until the point it is collision free
            else
            {
                if (q_near->getId() != coll.second->getId())
                {
                    // coll.second->setId(q_rand->getId());
                    // coll.second->setComponentId(q_rand->getComponentId());
                    coll.second->setId(node_id_++);
                    coll.second->setComponentId(q_near->getComponentId());
                    // ROS_INFO("q_near comp id: %d", q_near->getComponentId());
                    // ROS_INFO("coll.second comp id: %d", coll.second->getComponentId());

                    PRMgraph_.push_back(coll.second);

                    shared_ptr<Edge> collision_free_edge = make_shared<Edge>(q_near, coll.second);
                    // ROS_INFO("edge divisions %f", edge.getDivisions());

                    collision_free_edge->setDivisions(edge->getDivisions());
                    collision_free_edge->setTraversalTime(collision_free_edge->getDivisions() * timestep_);
                    // ROS_INFO("Traversal Time for REWRITTEN edge: %f \t %f", collision_free_edge->getDivisions(), collision_free_edge->getTraversalTime());
                    coll.second->addEdge(q_near, collision_free_edge);
                    q_near->addEdge(coll.second, collision_free_edge);

                }

                // No else needed because you don't need an edge to yourself.
                continue;
            }

            if(q_rand->getComponentId() != q_near->getComponentId())
            {
                // merge two components
                // q_rand->setComponentId(q_near->getComponentId());

                int overwrite_id = q_near->getComponentId();
                // ROS_INFO("overwrite id %d", overwrite_id);
                int new_id = q_rand->getComponentId();
                // ROS_INFO("new_id id %d", new_id);

                for (auto v : PRMgraph_)
                {
                    if (v->getComponentId() == overwrite_id)
                    {
                        v->setComponentId(new_id);
                    }
                }
            }
            // Have a neighbourhood, but can't connect to anyone
            // else
            // {
            //     q_rand->setComponentId(component_);
            //     component_++;
            // } 
        }
        
    }
    ROS_INFO("BuldPRM ends here");

}