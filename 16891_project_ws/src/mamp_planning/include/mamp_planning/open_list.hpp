#pragma once

#include <unordered_map>
#include <set>
#include <memory>
#include "mamp_planning/vertex.hpp"

template <class T, typename CompareT> class OpenList
{
public:
    OpenList();

    // pop - removes top element and returns it
    std::shared_ptr<T> pop();

    // checks to see if vertex is already in the open list, if so, replace it
    // use this function to also reorder the position of the vertex within the list
    bool insert(std::shared_ptr<T> v);

    // contains - checks to see if vertex is in open list
    bool contains(std::shared_ptr<T> v);

    size_t size();

private:
    std::unordered_map<unsigned int, std::shared_ptr<T>> check_list_;
    std::set<std::shared_ptr<T>, CompareT> ordered_list_;
};

// #include "mamp_planning/open_list.hpp"

template <class T, typename CompareT> OpenList<T, CompareT>::OpenList()
{
}

// pop - removes top element and returns it
template <class T, typename CompareT> std::shared_ptr<T> OpenList<T, CompareT>::pop()
{
  if (check_list_.size() == 0 || ordered_list_.size() == 0)
  {
    return nullptr;
  }
  std::shared_ptr<T> s = *(ordered_list_.begin());
  ordered_list_.erase(ordered_list_.begin());
  check_list_.erase(s->getId());
  return s;
}

// checks to see if vertex is already in the open list, if so, replace it, if not, insert
// use this function to also reorder the position of the vertex within the list
template <class T, typename CompareT> bool OpenList<T, CompareT>::insert(std::shared_ptr<T> v)
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
template <class T, typename CompareT> bool OpenList<T, CompareT>::contains(std::shared_ptr<T> v)
{
  return check_list_.find(v->getId()) != check_list_.end();
}

template <class T, typename CompareT> size_t OpenList<T, CompareT>::size()
{
    return check_list_.size();
}