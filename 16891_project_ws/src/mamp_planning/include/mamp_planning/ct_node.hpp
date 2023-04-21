#pragma once

#include "mamp_planning/agent.hpp"
#include "mamp_planning/mamp_helper.hpp"




class CTNode
{
public:
    CTNode(unsigned int id, std::unordered_map<std::string, std::shared_ptr<Agent>> agents, std::shared_ptr<MAMP_Helper> &mamp_helper);
    CTNode(unsigned int id, std::shared_ptr<CTNode> &n);
    //TODO: add constraints for each agent
    void addConstraint(Constraint c);
    std::vector<Collision> &getCollisions();
    void computeCost();
    std::vector<Constraint> &getConstraints();
    std::unordered_map<std::string, std::vector<std::shared_ptr<Vertex>>> &getPaths();
    std::unordered_map<std::string, std::shared_ptr<Agent>> &getAgents();
    std::tuple<double, size_t, unsigned int> getComparisonTuple();
    void detectCollisions();
    size_t numCollisions();
    unsigned int getId();
    std::shared_ptr<MAMP_Helper> &getMAMPHelper();
    double getMaxConstraintTime();
    double getCost();



private:
    double cost_;
    unsigned int id_;
    // std::vector<std::shared_ptr<Agent>> agents_;
    std::unordered_map<std::string, std::shared_ptr<Agent>> agents_;
    std::unordered_map<std::string, std::vector<std::shared_ptr<Vertex>>> paths_;
    std::vector<Collision> collisions_;
    std::vector<Constraint> constraints_;
    double max_constraint_time_;
    std::shared_ptr<MAMP_Helper> mamp_helper_;
    size_t num_collisions_;
};

struct CompareCTNode
{
    bool operator()(const std::shared_ptr<CTNode> &a, const std::shared_ptr<CTNode> &b) const
    {
        return a->getComparisonTuple() < b->getComparisonTuple();
    }
};