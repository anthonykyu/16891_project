#include "mamp_planning/prm.hpp"

PRM::PRM(std::shared_ptr<planning_scene::PlanningScene> planning_scene, double timestep, 
        std::vector<double> &jnt_vel_lim, std::vector<double> &jnt_upper_lim, std::vector<double> &jnt_lower_lim,
        std::shared_ptr<Vertex> start, std::shared_ptr<Vertex> goal)
{
  radius_ = 0.5;
  num_samples_ = 500;
  start_ = start;
  goal_ = goal;
  planning_scene_ = planning_scene;
  timestep_ = timestep;
  dof_ = jnt_upper_lim.size();
  node_id_ = 2;
  jnt_vel_lim_ = jnt_vel_lim;
  jnt_lower_lim_ = jnt_lower_lim;
  jnt_upper_lim_ = jnt_upper_lim;
  ROS_INFO("We have a new PRM!");

}

bool PRM::CheckCollision(vector<double> joint_pos)
{
    return false;
}

double PRM::GetDistance(shared_ptr<Vertex> v1, shared_ptr<Vertex> v2)
{
  double distance = 0;
  for (int i = 0; i < v1->getJointPos().size(); ++i)
  {
    double d = v1->getJointPos()[i] - v2->getJointPos()[i];
    distance += d*d;
  }
  return sqrt(distance);
}

void PRM::GetNeighbors(shared_ptr<Vertex> q_new, double radius)
{
  for (int i = 0; i < PRMgraph_.size(); ++i)
  {
    // Don't connect with yourself
    if (q_new->getId() == PRMgraph_[i]->getId())
    {
      continue;
    }

    // If the node is within range, then add it to my neighbors
    double distance = GetDistance(q_new, PRMgraph_[i]);
    if (distance <= radius)
    {
        q_new->addToNeighborhood(PRMgraph_[i]);
    }
  }
}

shared_ptr<Vertex> PRM::GetRandomVertex()
{
    // use Vertex class to create a new vertex
    vector<double> q_rand_pos(dof_);
    for (int i = 0; i < dof_; ++i)
    {
      q_rand_pos[i] = jnt_lower_lim_[i] + (((double)rand() / (double)RAND_MAX) * (jnt_upper_lim_[i] - jnt_lower_lim_[i])) ;
      printf("This is %f\n", q_rand_pos[i]);
    }
    shared_ptr<Vertex> q_rand = make_shared<Vertex>(q_rand_pos, node_id_++);
    printf("Now we have %ld\n", q_rand->getJointPos().size());

    // TODO: Shivani add the function to check if this is a valid set of joint positions, otherwise make a new set.
    // TODO: Get a vertex collision check, could be hitting an obstacle

    return q_rand;
}

shared_ptr<Vertex> PRM::GetNewVertex(shared_ptr<Vertex> q_near,shared_ptr<Vertex> q, int r)
{
    // create a new vertex
    double distance = GetDistance(q_near, q);
    int num_steps = (int)(distance / r);

    if (num_steps < 2)
    {
        shared_ptr<Vertex> q_new = make_shared<Vertex>(q->getJointPos(), q->getId());
        return q_new;
    }

    auto unit_vector = q_near->getJointPos();
    for (int i = 0; i < q->getJointPos().size(); ++i)
    {
        unit_vector[i] = (q->getJointPos()[i] - q_near->getJointPos()[i]) / distance;
        unit_vector[i] = q_near->getJointPos()[i] + unit_vector[i] * epsilon_;

    }
    shared_ptr<Vertex> q_new = make_shared<Vertex>(unit_vector, node_id_++);

    return q_new;

}

shared_ptr<Vertex> PRM::GetNearestVertex(shared_ptr<Vertex> q, vector<shared_ptr<Vertex>> nodes, int max_id)
{
    double min_distance = 1000000;
    shared_ptr<Vertex> q_near;
    for(int i = 0; i < nodes.size(); i++)
    {
        if (nodes[i]->getComponentId() != max_id)
        {
            continue;
        }

        double distance = GetDistance(q, nodes[i]);
        if (distance < min_distance)
        {
            min_distance = distance;
            q_near = nodes[i];
        }
    }
    return q_near;
}

bool PRM::Connect(shared_ptr<Vertex> q1, shared_ptr<Vertex> q2)
{
    double distance = GetDistance(q1, q2);
    int num_steps = (int)(distance / epsilon_);
    if (num_steps < 2)
    {
        return true;
    }

    
    for (int i = 0; i < num_steps; ++i)
    {
        // creat a double vector with the same size as the joint position
        vector<double> unit_vector;
        for (int j = 0; j < q1->getJointPos().size(); ++j)
        {
            unit_vector[j] = q1->getJointPos()[j] + (q2->getJointPos()[j] - q1->getJointPos()[j]) * i / (num_steps-1);        
        }

        // how do I check collision for these joint positionss in the world?        
        if (!CheckCollision(unit_vector)) 
        {
            return false;
        }
    }
    return false;
}

void PRM::GetPath(vector<shared_ptr<Vertex>> nodes)
{
    shared_ptr<Vertex> q = goal_;
    while (q->getId() != start_->getId())
    {
        PRMpath_.push_back(q);
        q = q->getParent();
    }
    PRMpath_.push_back(start_);
}

void PRM::BuildPRM()
{
    int component = 0;

    ROS_INFO("I got into the function");
    for (int i = 0; i < num_samples_; i++)
    {
        shared_ptr<Vertex> q_rand = GetRandomVertex();
        //NOTE: if the given configuration is valid, need to check with Hardik
        // if (CheckCollision(q_rand->getJointPos()))
        // {
        //     continue;
        // }

        ROS_INFO("~~~~ Iteration %d", i);

        PRMgraph_.push_back(q_rand);
        GetNeighbors(q_rand, radius_);

        ROS_INFO("We have NEIGHBOURS that we know.");

        // if the neighborhood is empty, then the new vertex is a new component
        if (q_rand->getNeighborhood().empty())
        {
            q_rand->setComponentId(component);
            component++;
            continue;
        }
        
        ROS_INFO("It has neighbors. Let's add edges.");

        for(int j = 0; j < q_rand->getNeighborhood().size(); j++)
        {
            shared_ptr<Vertex> q_near = q_rand->getNeighborhood()[j];
            ROS_INFO("Obtained a neighbor");

            // If you're already in the same component, you can skip everything else!
            if (q_rand->getComponentId() == q_near->getComponentId())
            {
                continue;
            }
            ROS_INFO("Got past componnent check");


            shared_ptr<Edge> edge = make_shared<Edge>(q_near, q_rand);
            ROS_INFO("Q_near has joints: %ld", q_near->getJointPos().size());
            auto coll = MAMP_Helper::detectEdgeCollision(planning_scene_, edge, jnt_vel_lim_, timestep_);

            ROS_INFO("Executed Collision Check");
            if (coll.first == false) // no collision
            {
                q_rand->addEdge(q_near, edge);
                q_near->addEdge(q_rand, edge);
                // PRMgraph_.push_back(q_rand);
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
                    PRMgraph_.push_back(coll.second);

                    shared_ptr<Edge> collision_free_edge = make_shared<Edge>(q_near, coll.second);
                    coll.second->addEdge(q_near, collision_free_edge);
                    q_near->addEdge(coll.second, collision_free_edge);

                }
                continue;
            }

            if(q_rand->getComponentId() != q_near->getComponentId())
            {
                // merge two components
                // q_rand->setComponentId(q_near->getComponentId());

                int overwrite_id = q_near->getComponentId();
                int new_id = q_rand->getComponentId();

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
            //     q_rand->setComponentId(component);
            //     component++;
            // } 
        }
        
    }
}