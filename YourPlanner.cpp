#include "YourPlanner.h"
#include "RrtConConBase.h"
#include <rl/plan/SimpleModel.h>
#include <rl/math/Vector.h>
#include <iostream>
#include <rl/plan/Sampler.h>
#include <rl/plan/Verifier.h>
#include <rl/plan/Viewer.h>
#include <boost/make_shared.hpp>




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


RrtConConBase::Neighbor
YourPlanner::nearest(const Tree& tree, const ::rl::math::Vector& chosen)
{
  // Erstellen Sie eine leere Liste von Knoten, die ausgeschlossen werden sollen
  std::set<Vertex> exclude_list;

  // Erstellen Sie ein leeres Paar <Vertex, distance> zur Rückgabe
  Neighbor p(Vertex(), (::std::numeric_limits< ::rl::math::Real >::max)());

  // Iterieren Sie durch alle Knoten, um den nächsten Nachbarn zu finden
  for (VertexIteratorPair i = ::boost::vertices(tree); i.first != i.second; ++i.first)
  {
    // Überprüfen Sie, ob der Knoten in der exclude_list ist
    if (exclude_list.find(*i.first) != exclude_list.end())
    {
      continue;
    }

    // Berechnen Sie die Distanz
    ::rl::math::Real d = this->model->transformedDistance(chosen, *tree[*i.first].q);

    if (d < p.second)
    {
      p.first = *i.first;
      p.second = d;
    }

    // Überprüfen Sie, ob der Knoten zu weit entfernt ist
    if (d - this->delta > p.second)
    {
      // Fügen Sie den Knoten und seine Kinder zur exclude_list hinzu
      std::pair<Tree::adjacency_iterator, Tree::adjacency_iterator> children = boost::adjacent_vertices(*i.first, tree);
      for (Tree::adjacency_iterator it = children.first; it != children.second; ++it)
      {
        exclude_list.insert(*it);
      }
    }
  }

  // Berechnen Sie die Quadratwurzel der Distanz
  p.second = this->model->inverseOfTransformedDistance(p.second);

  return p;
}


bool
YourPlanner::solve()
{
  //your modifications here
  return RrtConConBase::solve();
}

