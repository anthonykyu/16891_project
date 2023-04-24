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

template <typename ComparisonTuple, typename IDTuple, class T, typename hashT>
class OpenList
{
public:
  OpenList();
  OpenList(const OpenList &l);

  // pop - removes top element and returns it
  std::tuple<ComparisonTuple, IDTuple, std::shared_ptr<T>> pop();

  // top - returns top element
  std::tuple<ComparisonTuple, IDTuple, std::shared_ptr<T>> top();
   // pop element - removes a specific element and returns it
  // std::tuple<ComparisonTuple, IDTuple, std::shared_ptr<T>> remove(std::shared_ptr<T> t);

  void remove(IDTuple t);


  // checks to see if vertex is already in the open list, if so, replace it
  // use this function to also reorder the position of the vertex within the list
  bool insert(ComparisonTuple ct, IDTuple idt, std::shared_ptr<T> v);

  // contains - checks to see if vertex is in open list
  bool contains(IDTuple t);

  size_t size();

  void clear();

  // std::unordered_map<IDTuple, std::shared_ptr<T>, hashT> getCheckList();
  // std::unordered_map<IDTuple, ComparisonTuple, hashT> getTupleMap();
  // std::map<ComparisonTuple, IDTuple> getOrderedList();

private:
  std::unordered_map<IDTuple, std::shared_ptr<T>, hashT> check_list_;
  std::unordered_map<IDTuple, ComparisonTuple, hashT> tuple_map_;
  std::map<ComparisonTuple, IDTuple> ordered_list_;
};

// #include "mamp_planning/open_list.hpp"

template <typename ComparisonTuple, typename IDTuple, class T, typename hashT>
OpenList<ComparisonTuple, IDTuple, T, hashT>::OpenList()
{
}

template <typename ComparisonTuple, typename IDTuple, class T, typename hashT>
OpenList<ComparisonTuple, IDTuple, T, hashT>::OpenList(const OpenList &l)
{
  // printf("Check1.1");
  this->check_list_ = l.check_list_;
  // printf("Check1.2");
  this->tuple_map_ = l.tuple_map_;
  // printf("Check1.3");
  this->ordered_list_ = l.ordered_list_;
  // printf("Check1.4");
}

// template <typename ComparisonTuple, typename IDTuple, class T, typename hashT>
// std::unordered_map<IDTuple, std::shared_ptr<T>, hashT> OpenList<ComparisonTuple, IDTuple, T, hashT>::getCheckList()
// {
//   return check_list_;
// }

// template <typename ComparisonTuple, typename IDTuple, class T, typename hashT>
// std::unordered_map<IDTuple, ComparisonTuple, hashT> OpenList<ComparisonTuple, IDTuple, T, hashT>::getTupleMap()
// {
//   return tuple_map_;
// }

// template <typename ComparisonTuple, typename IDTuple, class T, typename hashT>
// std::map<ComparisonTuple, IDTuple> OpenList<ComparisonTuple, IDTuple, T, hashT>::getOrderedList()
// {
//   return ordered_list_;
// }

template <typename ComparisonTuple, typename IDTuple, class T, typename hashT>
void OpenList<ComparisonTuple, IDTuple, T, hashT>::clear()
{
  check_list_.clear();
  ordered_list_.clear();
  tuple_map_.clear();
}

// pop - removes top element and returns it
template <typename ComparisonTuple, typename IDTuple, class T, typename hashT>
std::tuple<ComparisonTuple, IDTuple, std::shared_ptr<T>> OpenList<ComparisonTuple, IDTuple, T, hashT>::pop()
{
  if (check_list_.size() == 0 || ordered_list_.size() == 0)
  {
    return std::tuple<ComparisonTuple, IDTuple, std::shared_ptr<T>>();
  }
  auto s = ordered_list_.begin();
  std::tuple<ComparisonTuple, IDTuple, std::shared_ptr<T>> v = std::make_tuple(s->first, s->second, check_list_[s->second]);
  ordered_list_.erase(s);
  check_list_.erase(s->second);
  tuple_map_.erase(s->second);
  return v;
}

// top - returns top element
template <typename ComparisonTuple, typename IDTuple, class T, typename hashT>
std::tuple<ComparisonTuple, IDTuple, std::shared_ptr<T>> OpenList<ComparisonTuple, IDTuple, T, hashT>::top()
{
  if (check_list_.size() == 0 || ordered_list_.size() == 0)
  {
    return std::tuple<ComparisonTuple, IDTuple, std::shared_ptr<T>>();
  }
  auto s = ordered_list_.begin();
  std::tuple<ComparisonTuple, IDTuple, std::shared_ptr<T>> v = std::make_tuple(s->first, s->second, check_list_[s->second]);
  return v;
}

//pop - removes a specific element and returns it
// template <typename ComparisonTuple, typename IDTuple, class T, typename hashT>
// std::pair<ComparisonTuple, typename IDTuple, std::shared_ptr<T>> OpenList<ComparisonTuple, typename IDTuple, T, hashT>::pop_element(std::shared_ptr<T> element)
// {
//   if (check_list_.size() == 0 || ordered_list_.size() == 0)
//   {
//     return std::pair<ComparisonTuple, typename IDTuple, std::shared_ptr<T>>();
//   }
//   // find the element 'element'  in the ordered list 
//   if (contains(element))
//   {
//     auto s = check_list_.find(element);
//     std::pair<ComparisonTuple, typename IDTuple, std::shared_ptr<T>> v = std::make_pair(s->first, s->second);
//     ordered_list_.erase(s);
//     check_list_.erase(s->first);
//     return v;
//   }



  // if (ordered_list_.find(element) == ordered_list_.end())
  // {
  //   return std::pair<ComparisonTuple, typename IDTuple, std::shared_ptr<T>>();
  // }
  // else 
  // {
  //   auto s = ordered_list_.find(element);
  //   std::pair<ComparisonTuple, typename IDTuple, std::shared_ptr<T>> v = std::make_pair(s->first, s->second);
  //   ordered_list_.erase(s);
  //   check_list_.erase(s->first);
  //   return v;
  // }
// }

// template <typename ComparisonTuple, typename IDTuple, class T, typename hashT>
// std::tuple<ComparisonTuple, IDTuple, std::shared_ptr<T>> OpenList<ComparisonTuple, IDTuple, T, hashT>::remove(IDTuple t)
// {
//   if (check_list_.size() == 0 || ordered_list_.size() == 0)
//   {
//     return std::pair<ComparisonTuple, typename IDTuple, std::shared_ptr<T>>();
//   }

//   // loop over the ordered list and find the key corresponding to the input element
//   ComparisonTuple, IDTuple key;
//   for (auto it = ordered_list_.begin(); it != ordered_list_.end(); ++it)
//   {
//     if ((it->second)->getId() == element->getId()) // compare the std::shared_ptr<T> values
//     {
//       key = it->first; // save the corresponding key
//       break;
//     }
//   }

//   // check if we found a valid key
//   if (key == ComparisonTuple, typename IDTuple())
//   {
//     return std::pair<ComparisonTuple, typename IDTuple, std::shared_ptr<T>>();
//   }
//   else 
//   {
//     // remove the key-value pair from the ordered list and check list
//     std::pair<ComparisonTuple, typename IDTuple, std::shared_ptr<T>> v = std::make_pair(key, ordered_list_[key]);
//     ordered_list_.erase(key);
//     check_list_.erase(key);
//     return v;
//   }
// }


// checks to see if vertex is already in the open list, if so, replace it, if not, insert
// use this function to also reorder the position of the vertex within the list
template <typename ComparisonTuple, typename IDTuple, class T, typename hashT>
bool OpenList<ComparisonTuple, IDTuple, T, hashT>::insert(ComparisonTuple ct, IDTuple idt, std::shared_ptr<T> v)
{
  bool success = true;
  if (contains(idt))
  {
    ordered_list_.erase(tuple_map_[idt]);
    tuple_map_.erase(idt);
    success = success && tuple_map_.insert({idt, ct}).second;
    success = success && ordered_list_.insert({ct, idt}).second;
  }
  else
  {
    success = success && tuple_map_.insert({idt, ct}).second;
    success = success && check_list_.insert({idt, v}).second;
    success = success && ordered_list_.insert({ct, idt}).second;
  }
  return success;
}

template <typename ComparisonTuple, typename IDTuple, class T, typename hashT>
void OpenList<ComparisonTuple, IDTuple, T, hashT>::remove(IDTuple t)
{
  if (contains(t))
  {
    check_list_.erase(t);
    ordered_list_.erase(tuple_map_[t]);
    tuple_map_.erase(t);
  }
}

// contains - checks to see if vertex is in open list
template <typename ComparisonTuple, typename IDTuple, class T, typename hashT>
bool OpenList<ComparisonTuple, IDTuple, T, hashT>::contains(IDTuple t)
{
  return check_list_.find(t) != check_list_.end();
}

template <typename ComparisonTuple, typename IDTuple, class T, typename hashT>
size_t OpenList<ComparisonTuple, IDTuple, T, hashT>::size()
{
  return check_list_.size();
}