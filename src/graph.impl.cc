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

#include <hpp/manipulation/problem.hh>
#include <hpp/manipulation/roadmap.hh>
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
        graph_->maxIterations (problemSolver_->maxIterations ());
        graph_->errorThreshold (problemSolver_->errorThreshold ());
        problemSolver_->constraintGraph (graph_);
        problemSolver_->problem()->constraintGraph (graph_);
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

      Long Graph::createNode(const Long subgraphId, const char* nodeName)
        throw (hpp::Error)
      {
        graph::NodeSelectorPtr_t ns;
        try {
          ns = HPP_DYNAMIC_PTR_CAST(graph::NodeSelector,
              graph::GraphComponent::get(subgraphId).lock());
        } catch (std::out_of_range& e) {
          throw Error (e.what());
        }
        if (!ns)
          throw Error ("You should create a subgraph "
              " before creating nodes.");

        graph::NodePtr_t node = ns->createNode ();
        node->name (nodeName);
        return node->id ();
      }

      Long Graph::createEdge(const Long nodeFromId, const Long nodeToId, const char* edgeName, const Long w, const bool isInNodeFrom)
        throw (hpp::Error)
      {
        graph::NodePtr_t from, to;
        try {
          from = HPP_DYNAMIC_PTR_CAST(graph::Node, graph::GraphComponent::get(nodeFromId).lock());
          to   = HPP_DYNAMIC_PTR_CAST(graph::Node, graph::GraphComponent::get(nodeToId  ).lock());
        } catch (std::out_of_range& e) {
          throw Error (e.what());
        }
        if (!from || !to)
          throw Error ("The nodes could not be found.");

        graph::EdgePtr_t edge = from->linkTo (to, w, isInNodeFrom);
        edge->name (edgeName);
        return edge->id ();
      }

      void Graph::setNumericalConstraints (const Long graphComponentId,
          const hpp::Names_t& constraintNames)
        throw (hpp::Error)
      {
        graph::GraphComponentPtr_t component = graph::GraphComponent::get(graphComponentId).lock();
        if (!component)
          throw Error ("The ID does not exist.");

        if (constraintNames.length () > 0) {
          try {
            for (CORBA::ULong i=0; i<constraintNames.length (); ++i) {
              std::string name (constraintNames [i]);
              component->addNumericalConstraint (problemSolver_->numericalConstraint(name));
            }
          } catch (std::exception& err) {
            throw Error (err.what());
          }
        }
      }

      void Graph::setNumericalConstraintsForPath (const Long nodeId,
          const hpp::Names_t& constraintNames)
        throw (hpp::Error)
      {
        graph::NodePtr_t n;
        try {
          n = HPP_DYNAMIC_PTR_CAST(graph::Node, graph::GraphComponent::get(nodeId).lock());
        } catch (std::out_of_range& e) {
          throw Error (e.what());
        }
        if (!n)
          throw Error ("The nodes could not be found.");

        if (constraintNames.length () > 0) {
          try {
            for (CORBA::ULong i=0; i<constraintNames.length (); ++i) {
              std::string name (constraintNames [i]);
              n->addNumericalConstraintForPath (problemSolver_->numericalConstraint(name));
            }
          } catch (std::exception& err) {
            throw Error (err.what());
          }
        }
      }

      void Graph::setLockedDofConstraints (const Long graphComponentId,
          const hpp::Names_t& constraintNames)
        throw (hpp::Error)
      {
        graph::GraphComponentPtr_t component = graph::GraphComponent::get(graphComponentId).lock();
        if (!component)
          throw Error ("The ID does not exist.");

        if (constraintNames.length () > 0) {
          try {
            for (CORBA::ULong i=0; i<constraintNames.length (); ++i) {
              std::string name (constraintNames [i]);
              component->addLockedDofConstraint (problemSolver_->lockedDofConstraint(name));
            }
          } catch (std::exception& err) {
            throw Error (err.what());
          }
        }
      }

      void Graph::statOnConstraint (const hpp::IDseq& IDedges)
        throw (hpp::Error)
      {
        graph::Edges_t edges;
        graph::EdgePtr_t edge;
        for (CORBA::ULong i=0; i < IDedges.length (); ++i) {
          size_t id (IDedges [i]);
          try {
            edge = HPP_DYNAMIC_PTR_CAST(graph::Edge,
                graph::GraphComponent::get(id).lock ());
          } catch (std::exception& e ) {
            throw Error (e.what());
          }
          if (!edge) {
            std::stringstream ss;
            ss << "ID " << id << " is not an edge";
            std::string errmsg = ss.str();
            throw Error (errmsg.c_str());
          }
          edges.push_back (edge);
        }
        try {
          RoadmapPtr_t roadmap = HPP_DYNAMIC_PTR_CAST (Roadmap, problemSolver_->roadmap());
          if (!roadmap)
            throw Error ("The roadmap is not of type hpp::manipulation::Roadmap.");
          roadmap->statAddFoliation (graph_->configConstraint (edges));
        } catch (std::exception& e) {
          throw Error (e.what());
        }
      }

      void Graph::getNodes (const hpp::floatSeq& dofArray, IDseq_out output)
        throw (hpp::Error)
      {
        try {
          vector_t config; config.resize (dofArray.length());
          for (int iDof = 0; iDof < config.size(); iDof++) {
            config [iDof] = dofArray[iDof];
          }
          graph::Nodes_t nodes = graph_->getNode (config);
          IDseq* ret_ptr = new IDseq ();
          ret_ptr->length(nodes.size());
          size_t s = 0;
          graph::Nodes_t::iterator it;
          for (it = nodes.begin (); it != nodes.end(); it++) {
            (*ret_ptr)[s] = (*it)->id();
            s++;
          }
          output = ret_ptr;
        } catch (std::exception& e) {
          throw Error (e.what());
        }
      }

      void Graph::display ()
        throw (hpp::Error)
      {
        std::cout << *graph_;
      }
    } // namespace impl
  } // namespace manipulation
} // namespace hpp
