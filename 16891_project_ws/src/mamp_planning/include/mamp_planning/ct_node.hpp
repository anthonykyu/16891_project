#pragma once

#include "mamp_planning/agent.hpp"
#include "mamp_planning/mamp_helper.hpp"




class CTNode
{
public:
    CTNode(unsigned int id, std::vector<std::shared_ptr<Agent>> agents);
    CTNode(unsigned int id, std::shared_ptr<CTNode> &n);
    //TODO: add constraints for each agent
    void addConstraint(Constraint c);
    Collision &getNextCollision();
    void computeCost();
    std::vector<Constraint> const &getConstraints();
    std::unordered_map<unsigned int, std::vector<std::shared_ptr<Vertex>>> const &getPaths();
    std::vector<std::shared_ptr<Agent>> &getAgents();
    std::tuple<double, unsigned int> getComparisonTuple();



private:
    double cost_;
    unsigned int id_;
    std::vector<std::shared_ptr<Agent>> agents_;
    std::unordered_map<unsigned int, std::vector<std::shared_ptr<Vertex>>> paths_;
    std::vector<Collision> collisions_;
    std::vector<Constraint> constraints_;

};

struct CompareCTNode
{
    bool operator()(const std::shared_ptr<CTNode> &a, const std::shared_ptr<CTNode> &b) const
    {
        return a->getComparisonTuple() < b->getComparisonTuple();
    }
};