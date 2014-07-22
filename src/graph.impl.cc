// Copyright (c) 2014, LAAS-CNRS
// Authors: Joseph Mirabel (joseph.mirabel@laas.fr)
//
// This file is part of hpp-manipulation.
// hpp-manipulation is free software: you can redistribute it
// and/or modify it under the terms of the GNU Lesser General Public
// License as published by the Free Software Foundation, either version
// 3 of the License, or (at your option) any later version.
//
// hpp-manipulation is distributed in the hope that it will be
// useful, but WITHOUT ANY WARRANTY; without even the implied warranty
// of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
// General Lesser Public License for more details.  You should have
// received a copy of the GNU Lesser General Public License along with
// hpp-manipulation. If not, see <http://www.gnu.org/licenses/>.

#include <hpp/util/debug.hh>
#include <hpp/util/pointer.hh>

#include <hpp/manipulation/graph/node-selector.hh>
#include <hpp/manipulation/graph/node.hh>
#include <hpp/manipulation/graph/graph.hh>
#include <hpp/manipulation/graph/edge.hh>

#include "graph.impl.hh"

namespace hpp {
  namespace manipulation {
    namespace impl {
      Graph::Graph () :
        problemSolver_ (0x0), graph_ ()
      {}

      Long Graph::createGraph(const char* graphName)
        throw (hpp::Error)
      {
        RobotPtr_t robot = problemSolver_->robot ();
        if (!robot) {
          throw Error ("You should build a composite robot"
              " before creating a graph.");
        }
        graph_ = graph::Graph::create(robot);
        graph_->name(graphName);
        return graph_->id ();
      }

      Long Graph::createSubGraph(const char* subgraphName)
        throw (hpp::Error)
      {
        if (!graph_)
          throw Error ("You should create the graph"
              " before creating subgraph.");
        graph::NodeSelectorPtr_t ns = graph_->createNodeSelector();
        ns->name(subgraphName);
        return ns->id ();
      }

      Long Graph::createNode(const Long subgraphId, const char* nodeName, const char* constraintName)
        throw (hpp::Error)
      {
        graph::NodeSelectorPtr_t ns =
          HPP_DYNAMIC_PTR_CAST(graph::NodeSelector,
              graph::GraphComponent::get(subgraphId).lock());
        if (!ns)
          throw Error ("You should create a subgraph "
            " before creating nodes.");

        // TODO: A better way of associating constraint should be thought of.
        DifferentiableFunctionPtr_t df = problemSolver_->numericalConstraint(constraintName);
        ConfigProjectorPtr_t constraint = ConfigProjector::create (
            problemSolver_->robot(),
            constraintName,
            problemSolver_->errorThreshold(),
            problemSolver_->maxIterations());
        constraint->addConstraint (df);
        graph::NodePtr_t node = ns->createNode (constraint);
        node->name (nodeName);
        return node->id ();
      }

      Long Graph::createEdge(const Long nodeFromId, const Long nodeToId, const char* edgeName, const char* constraintName)
        throw (hpp::Error)
      {
        graph::NodePtr_t from = HPP_DYNAMIC_PTR_CAST(graph::Node, graph::GraphComponent::get(nodeFromId).lock());
        graph::NodePtr_t to   = HPP_DYNAMIC_PTR_CAST(graph::Node, graph::GraphComponent::get(nodeToId  ).lock());
        if (!from || !to)
          throw Error ("The nodes could not be found.");

        // TODO: A better way of associating constraint should be thought of.
        DifferentiableFunctionPtr_t df = problemSolver_->numericalConstraint(constraintName);
        ConfigProjectorPtr_t constraint = ConfigProjector::create (
            problemSolver_->robot(),
            constraintName,
            problemSolver_->errorThreshold(),
            problemSolver_->maxIterations());
        constraint->addConstraint (df);
        graph::EdgePtr_t edge = from->linkTo (to, constraint);
        edge->name (edgeName);
        return edge->id ();
      }
    } // namespace impl
  } // namespace manipulation
} // namespace hpp
