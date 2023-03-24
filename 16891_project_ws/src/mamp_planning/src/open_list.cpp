#include "mamp_planning/open_list.hpp"

OpenList::OpenList()
{
}

// pop - removes top element and returns it
std::shared_ptr<Vertex> OpenList::pop()
{
  if (check_list_.size() == 0 || ordered_list_.size() == 0)
  {
    return nullptr;
  }
  std::shared_ptr<Vertex> s = *(ordered_list_.begin());
  ordered_list_.erase(ordered_list_.begin());
  check_list_.erase(s->getId());
  return s;
}

// checks to see if vertex is already in the open list, if so, replace it, if not, insert
// use this function to also reorder the position of the vertex within the list
bool OpenList::insert(std::shared_ptr<Vertex> v)
{
  bool success = true;
  if (contains(v))
  {
    ordered_list_.erase(v);
    success = success && ordered_list_.insert(v).second;
  }
  else
  {
    success = success && check_list_.insert({v->getId(), v}).second;
    success = success && ordered_list_.insert(v).second;
  }
  return success;
}

// contains - checks to see if vertex is in open list
bool OpenList::contains(std::shared_ptr<Vertex> v)
{
  return check_list_.find(v->getId()) != check_list_.end();
}