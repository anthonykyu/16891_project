#pragma once

#include <unordered_map>
#include <set>
#include <map>
#include <memory>
#include <tuple>
#include <utility>
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
  std::pair<Ttuple, std::shared_ptr<T>> pop();

  // top - returns top element
  std::pair<Ttuple, std::shared_ptr<T>> top();
   // pop element - removes a specific element and returns it
  std::pair<Ttuple, std::shared_ptr<T>> pop_element(std::shared_ptr<T> t);


  // checks to see if vertex is already in the open list, if so, replace it
  // use this function to also reorder the position of the vertex within the list
  bool insert(Ttuple t, std::shared_ptr<T> v);

  // contains - checks to see if vertex is in open list
  bool contains(Ttuple t);

  size_t size();

  void clear();

private:
  std::unordered_map<Ttuple, std::shared_ptr<T>, hashT> check_list_;
  std::map<Ttuple, std::shared_ptr<T>> ordered_list_;
};

// #include "mamp_planning/open_list.hpp"

template <typename Ttuple, class T, typename hashT>
OpenList<Ttuple, T, hashT>::OpenList()
{
}

template <typename Ttuple, class T, typename hashT>
void OpenList<Ttuple, T, hashT>::clear()
{
  check_list_.clear();
  ordered_list_.clear();
}

// pop - removes top element and returns it
template <typename Ttuple, class T, typename hashT>
std::pair<Ttuple, std::shared_ptr<T>> OpenList<Ttuple, T, hashT>::pop()
{
  if (check_list_.size() == 0 || ordered_list_.size() == 0)
  {
    return std::pair<Ttuple, std::shared_ptr<T>>();
  }
  auto s = ordered_list_.begin();
  std::pair<Ttuple, std::shared_ptr<T>> v = std::make_pair(s->first, s->second);
  ordered_list_.erase(s);
  check_list_.erase(s->first);
  return v;
}

// top - returns top element
template <typename Ttuple, class T, typename hashT>
std::pair<Ttuple, std::shared_ptr<T>> OpenList<Ttuple, T, hashT>::top()
{
  if (check_list_.size() == 0 || ordered_list_.size() == 0)
  {
    return std::pair<Ttuple, std::shared_ptr<T>>();
  }
  auto s = ordered_list_.begin();
  std::pair<Ttuple, std::shared_ptr<T>> v = std::make_pair(s->first, s->second);
  return v;
}

//pop - removes a specific element and returns it
// template <typename Ttuple, class T, typename hashT>
// std::pair<Ttuple, std::shared_ptr<T>> OpenList<Ttuple, T, hashT>::pop_element(std::shared_ptr<T> element)
// {
//   if (check_list_.size() == 0 || ordered_list_.size() == 0)
//   {
//     return std::pair<Ttuple, std::shared_ptr<T>>();
//   }
//   // find the element 'element'  in the ordered list 
//   if (contains(element))
//   {
//     auto s = check_list_.find(element);
//     std::pair<Ttuple, std::shared_ptr<T>> v = std::make_pair(s->first, s->second);
//     ordered_list_.erase(s);
//     check_list_.erase(s->first);
//     return v;
//   }



  // if (ordered_list_.find(element) == ordered_list_.end())
  // {
  //   return std::pair<Ttuple, std::shared_ptr<T>>();
  // }
  // else 
  // {
  //   auto s = ordered_list_.find(element);
  //   std::pair<Ttuple, std::shared_ptr<T>> v = std::make_pair(s->first, s->second);
  //   ordered_list_.erase(s);
  //   check_list_.erase(s->first);
  //   return v;
  // }
// }

template <typename Ttuple, class T, typename hashT>
std::pair<Ttuple, std::shared_ptr<T>> OpenList<Ttuple, T, hashT>::pop_element(std::shared_ptr<T> element)
{
  if (check_list_.size() == 0 || ordered_list_.size() == 0)
  {
    return std::pair<Ttuple, std::shared_ptr<T>>();
  }

  // loop over the ordered list and find the key corresponding to the input element
  Ttuple key;
  for (auto it = ordered_list_.begin(); it != ordered_list_.end(); ++it)
  {
    if ((it->second)->getId() == element->getId()) // compare the std::shared_ptr<T> values
    {
      key = it->first; // save the corresponding key
      break;
    }
  }

  // check if we found a valid key
  if (key == Ttuple())
  {
    return std::pair<Ttuple, std::shared_ptr<T>>();
  }
  else 
  {
    // remove the key-value pair from the ordered list and check list
    std::pair<Ttuple, std::shared_ptr<T>> v = std::make_pair(key, ordered_list_[key]);
    ordered_list_.erase(key);
    check_list_.erase(key);
    return v;
  }
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