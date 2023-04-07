#include "mamp_planning/prm.hpp"

PRM::PRM(std::shared_ptr<planning_scene::PlanningScene> planning_scene, double timestep, 
        std::vector<double> &jnt_vel_lim, std::vector<double> &jnt_upper_lim, std::vector<double> &jnt_lower_lim,
        std::shared_ptr<Vertex> start, std::shared_ptr<Vertex> goal)
{
  radius_ = 0.5;
  num_samples_ = 15000;
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

void PRM::GetNeighbors(shared_ptr<Vertex> q_new, vector<shared_ptr<Vertex>> graph, double radius)
{
  for (int i = 0; i < graph.size(); ++i)
  {
    if (q_new->getId() == graph[i]->getId())
    {
      continue;
    }
    shared_ptr<Vertex> q_near = graph[i];
    double distance = GetDistance(q_new, q_near);
    if (distance <= radius)
    {
      q_new->setNeighborhood(q_near->getNeighborhood());
    }
  }
}

shared_ptr<Vertex> PRM::GetRandomVertex(int dof)
{
    shared_ptr<Vertex> q_rand;
    auto q_rand_pos = q_rand->getJointPos();
    for (int i = 0; i < dof; ++i)
    {
      q_rand_pos[i] = (double)rand() / (double)RAND_MAX;
    }
    return q_rand;
}

shared_ptr<Vertex> PRM::GetNewVertex(shared_ptr<Vertex> q_near,shared_ptr<Vertex> q, int r)
{
    // create a new vertex
    shared_ptr<Vertex> q_new = make_shared<Vertex>(q_new->getJointPos(), q_new->getId());
    double distance = GetDistance(q_near, q);
    int num_steps = (int)(distance / r);

    if (num_steps < 2)
    {
        q_new = q;
        return q_new;
    }

    auto unit_vector = q_near->getJointPos();
    for (int i = 0; i < q->getJointPos().size(); ++i)
    {
        unit_vector[i] = (q->getJointPos()[i] - q_near->getJointPos()[i]) / distance;
        unit_vector[i] = q_near->getJointPos()[i] + unit_vector[i] * epsilon_;

    }
    q_new->setJointPos(unit_vector);
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
        if (!CheckCollision(unit_vector))
        {
            return false;
        }
    }
    return false;
}

void PRM::GetPath(shared_ptr<Vertex> q_start, shared_ptr<Vertex> q_goal, vector<shared_ptr<Vertex>> nodes)
{
    shared_ptr<Vertex> q = q_goal;
    while (q->getId() != q_start->getId())
    {
        PRMpath_.push_back(q);
        q = q->getParent();
    }
    PRMpath_.push_back(q_start);
}

void PRM::BuildPRM()
{
    int component = 0;
    for (int i = 0; i < num_samples_; i++)
    {
        shared_ptr<Vertex> q_rand = GetRandomVertex(dof_);
        if (CheckCollision(q_rand->getJointPos()))
        {
            continue;
        }

        q_rand->setId(PRMgraph_.size());
        PRMgraph_.push_back(q_rand);
        GetNeighbors(q_rand, PRMgraph_, radius_);

        // if the neighborhood is empty, then the new vertex is a new component
        if (q_rand->getNeighborhood().empty())
        {
            q_rand->setComponentId(component);
            component++;
            continue;
        }
        
        // if the neighborhood is not empty, then the new vertex is in the same component as its nearest neighbor
        for(int j = 0; j < q_rand->getNeighborhood().size(); j++)
        {
                shared_ptr<Vertex> q_near = q_rand->getNeighborhood()[j];
                if(q_rand->getComponentId() != q_near->getComponentId())
                {
                    // merge two components
                    if (Connect(q_rand, q_near))
                    {
                        q_rand->setComponentId(q_near->getComponentId());
                        shared_ptr<Edge> edge = make_shared<Edge>(q_rand, q_near);
                        q_rand->addEdge(q_near, edge);
                        q_near->addEdge(q_rand, edge);
                        for (int k = 0; k < PRMgraph_.size(); ++k)
                        {
                                PRMgraph_[k]->setComponentId(q_rand->getComponentId());    
                        }
                    }
                    else
                    {
                        q_rand->setComponentId(component);
                        component++;
                    } 
            }
        }
        
    }
}