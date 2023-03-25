#include "mamp_planning/prm.hpp"

PRM::PRM()
{
  radius_ = 0.5;
}

bool PRM::PRMCheckCollision(vector<double> joint_pos)
{
    return false;
}

double PRM::PRMGetDistance(shared_ptr<Vertex> v1, shared_ptr<Vertex> v2)
{
  double distance = 0;
  for (int i = 0; i < v1->getJointPos().size(); ++i)
  {
    double d = v1->getJointPos()[i] - v2->getJointPos()[i];
    distance += d*d;
  }
  return sqrt(distance);
}

void PRM::PRMGetNeighbors(shared_ptr<Vertex> q_new, vector<shared_ptr<Vertex>> graph, double radius)
{
  for (int i = 0; i < graph.size(); ++i)
  {
    if (q_new->getId() == graph[i]->getId())
    {
      continue;
    }
    shared_ptr<Vertex> q_near = graph[i];
    double distance = PRMGetDistance(q_new, q_near);
    if (distance <= radius)
    {
      neighborhood_.push_back(q_near);
    }
  }
}

shared_ptr<Vertex> PRM::PRMGetRandomVertex(int dof)
{
    shared_ptr<Vertex> q_rand;
    auto q_rand_pos = q_rand->getJointPos();
    for (int i = 0; i < dof; ++i)
    {
      q_rand_pos[i] = (double)rand() / (double)RAND_MAX;
    }
}

shared_ptr<Vertex> PRM::PRMGetNewVertex(shared_ptr<Vertex> q_near,shared_ptr<Vertex> q, int r)
{
    // create a new vertex
    shared_ptr<Vertex> q_new = make_shared<Vertex>(q_new->getJointPos(), q_new->getId());
    double distance = PRMGetDistance(q_near, q);
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

shared_ptr<Vertex> PRM::PRMGetNearestVertex(shared_ptr<Vertex> q, vector<shared_ptr<Vertex>> nodes, int max_id)
{
    double min_distance = 1000000;
    shared_ptr<Vertex> q_near;
    for(int i = 0; i < nodes.size(); i++)
    {
        if (nodes[i]->getComponentId() != max_id)
        {
            continue;
        }

        double distance = PRMGetDistance(q, nodes[i]);
        if (distance < min_distance)
        {
            min_distance = distance;
            q_near = nodes[i];
        }
    }
    return q_near;
}

bool PRM::PRMConnect(shared_ptr<Vertex> q1, shared_ptr<Vertex> q2)
{
    double distance = PRMGetDistance(q1, q2);
    int num_steps = (int)(distance / epsilon_);
    if (num_steps < 2)
    {
        return true;
    }

    
    for (int i = 0; i < num_steps; ++i)
    {
        // creat a double vector with the same size as the joint position
        auto unit_vector = q1->getJointPos();
        for (int j = 0; j < q1->getJointPos().size(); ++j)
        {
            unit_vector[j] = q1->getJointPos()[j] + (q2->getJointPos()[j] - q1->getJointPos()[j]) * i / (num_steps-1);        
        }
        if (!PRMCheckCollision(unit_vector))
        {
            return false;
        }
    }
    return false;
}