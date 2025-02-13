#include "YourPlanner.h"
#include "RrtConConBase.h"
#include <rl/plan/SimpleModel.h>
#include <rl/math/Vector.h>
#include <iostream>
#include <rl/plan/Sampler.h>
#include <rl/plan/Verifier.h>
#include <rl/plan/Viewer.h>
#include <boost/make_shared.hpp>
#include <cmath>




YourPlanner::YourPlanner() :
  RrtConConBase()
{
}

YourPlanner::~YourPlanner()
{
}

::std::string
YourPlanner::getName() const
{
  return "Your Planner";
}

void
YourPlanner::choose(::rl::math::Vector& chosen)
{
  //your modifications here
  RrtConConBase::choose(chosen);
  chosen = this->sampler->generate();
}

RrtConConBase::Vertex 
YourPlanner::connect(Tree& tree, const Neighbor& nearest, const ::rl::math::Vector& chosen)
{
  //your modifications here
  return RrtConConBase::connect(tree, nearest, chosen);
}

RrtConConBase::Vertex 
YourPlanner::extend(Tree& tree, const Neighbor& nearest, const ::rl::math::Vector& chosen)
{
  //your modifications here
  return RrtConConBase::extend(tree, nearest, chosen);
}





void
YourPlanner::excludeDescendants(const Tree& tree, Vertex node, std::set<Vertex>& exclude_list, int generations)
{
  if (generations <= 0)
  {
    return;
  }

  std::pair<Tree::adjacency_iterator, Tree::adjacency_iterator> children = boost::adjacent_vertices(node, tree);
  for (Tree::adjacency_iterator it = children.first; it != children.second; ++it)
  {
    if (exclude_list.insert(*it).second) // only recurse if the node was newly inserted
    {
      excludeDescendants(tree, *it, exclude_list, generations - 1);
    }
  }
}



RrtConConBase::Neighbor
YourPlanner::nearest(const Tree& tree, const ::rl::math::Vector& chosen)
{
  int generations;
  // Use an exclude_list of vertivces to not compute the distance for
  std::set<Vertex> exclude_list;

  //create an empty pair <Vertex, distance> to return
  Neighbor p(Vertex(), (::std::numeric_limits< ::rl::math::Real >::max)());

  //Iterate through all vertices to find the nearest neighbour
  for (VertexIteratorPair i = ::boost::vertices(tree); i.first != i.second; ++i.first)
  {
    //Check if vertex is on exclude list
    if (exclude_list.find(*i.first) != exclude_list.end())
    {
      continue;
    }
    
    // compute distance
    ::rl::math::Real d = this->model->transformedDistance(chosen, *tree[*i.first].q);
    if (d < p.second)
    {
      p.first = *i.first;
      p.second = d;
    }
    
    // exclude all neighboring vertices that can't reach the sample (number of steps = generations to exclude)
    if (d - this->delta > p.second)
    {
      generations = floor(d - p.second / this->delta);
      
      excludeDescendants(tree, *i.first, exclude_list, generations);
    }
  }
  
  // Compute the square root of distance
  p.second = this->model->inverseOfTransformedDistance(p.second);

  return p;
}


bool
YourPlanner::solve()
{
  //your modifications here
  return RrtConConBase::solve();
}

