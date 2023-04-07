#pragma once

#include <unordered_map>
#include <set>
#include <map>
#include <memory>
#include "mamp_planning/vertex.hpp"

namespace hash_tuple
{

  template <typename TT>
  struct hash
  {
    size_t
    operator()(TT const &tt) const
    {
      return std::hash<TT>()(tt);
    }
  };

  namespace
  {
    template <class T>
    inline void hash_combine(std::size_t &seed, T const &v)
    {
      seed ^= hash_tuple::hash<T>()(v) + 0x9e3779b9 + (seed << 6) + (seed >> 2);
    }
  }

  namespace
  {
    // Recursive template code derived from Matthieu M.
    template <class Tuple, size_t Index = std::tuple_size<Tuple>::value - 1>
    struct HashValueImpl
    {
      static void apply(size_t &seed, Tuple const &tuple)
      {
        HashValueImpl<Tuple, Index - 1>::apply(seed, tuple);
        hash_combine(seed, std::get<Index>(tuple));
      }
    };

    template <class Tuple>
    struct HashValueImpl<Tuple, 0>
    {
      static void apply(size_t &seed, Tuple const &tuple)
      {
        hash_combine(seed, std::get<0>(tuple));
      }
    };
  }

  template <typename... TT>
  struct hash<std::tuple<TT...>>
  {
    size_t
    operator()(std::tuple<TT...> const &tt) const
    {
      size_t seed = 0;
      HashValueImpl<std::tuple<TT...>>::apply(seed, tt);
      return seed;
    }
  };
}

template <typename Ttuple, class T, typename hashT>
class OpenList
{
public:
  OpenList();

  // pop - removes top element and returns it
  std::shared_ptr<T> pop();

  // checks to see if vertex is already in the open list, if so, replace it
  // use this function to also reorder the position of the vertex within the list
  bool insert(Ttuple t, std::shared_ptr<T> v);

  // contains - checks to see if vertex is in open list
  bool contains(Ttuple t);

  size_t size();

private:
  std::unordered_map<Ttuple, std::shared_ptr<T>, hashT> check_list_;
  std::map<Ttuple, std::shared_ptr<T>> ordered_list_;
};

// #include "mamp_planning/open_list.hpp"

template <typename Ttuple, class T, typename hashT>
OpenList<Ttuple, T, hashT>::OpenList()
{
}

// pop - removes top element and returns it
template <typename Ttuple, class T, typename hashT>
std::shared_ptr<T> OpenList<Ttuple, T, hashT>::pop()
{
  if (check_list_.size() == 0 || ordered_list_.size() == 0)
  {
    return nullptr;
  }
  auto s = ordered_list_.begin();
  ordered_list_.erase(s);
  check_list_.erase(s->first);
  return s->second;
}

// checks to see if vertex is already in the open list, if so, replace it, if not, insert
// use this function to also reorder the position of the vertex within the list
template <typename Ttuple, class T, typename hashT>
bool OpenList<Ttuple, T, hashT>::insert(Ttuple t, std::shared_ptr<T> v)
{
  bool success = true;
  if (contains(t))
  {
    ordered_list_.erase(t);
    success = success && ordered_list_.insert({t, v}).second;
  }
  else
  {
    success = success && check_list_.insert({t, v}).second;
    success = success && ordered_list_.insert({t, v}).second;
  }
  return success;
}

// contains - checks to see if vertex is in open list
template <typename Ttuple, class T, typename hashT>
bool OpenList<Ttuple, T, hashT>::contains(Ttuple t)
{
  return check_list_.find(t) != check_list_.end();
}

template <typename Ttuple, class T, typename hashT>
size_t OpenList<Ttuple, T, hashT>::size()
{
  return check_list_.size();
}