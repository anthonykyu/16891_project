#pragma once

#include <unordered_map>
#include <set>
#include <memory>
#include "mamp_planning/vertex.hpp"

class OpenList
{
public:
    OpenList();
    
    // pop - removes top element and returns it
    std::shared_ptr<Vertex> pop();

    // checks to see if vertex is already in the open list, if so, replace it
    // use this function to also reorder the position of the vertex within the list
    bool insert(std::shared_ptr<Vertex>);

    // contains - checks to see if vertex is in open list
    bool contains(std::shared_ptr<Vertex>);

private:
    std::unordered_map<unsigned int, std::shared_ptr<Vertex>> check_list_;
    std::set<std::shared_ptr<Vertex>, CompareVertex> ordered_list_;
};