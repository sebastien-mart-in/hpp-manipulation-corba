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

#ifndef HPP_MANIPULATION_CORBA_GRAPH_IMPL_HH
# define HPP_MANIPULATION_CORBA_GRAPH_IMPL_HH

# include <hpp/manipulation/problem-solver.hh>
# include <hpp/manipulation/graph/graph.hh>

# include "hpp/corbaserver/manipulation/fwd.hh"
# include "graph.hh"

namespace hpp {
  namespace manipulation {
    namespace impl {
      using CORBA::Short;

      class Graph : public virtual POA_hpp::corbaserver::manipulation::Graph
      {
        public:
          Graph ();
          void setProblemSolver (const ProblemSolverPtr_t& problemSolver)
          {
            problemSolver_ = problemSolver;
          }

          virtual Short createGraph(const char* graphName)
            throw (hpp::Error);

          virtual Short createSubGraph(const char* subgraphName)
            throw (hpp::Error);

          virtual Short createNode (const char* subGraphName,
                                   const char* nodeName,
                                   const char* constraintName)
            throw (hpp::Error);
        private:
          ProblemSolverPtr_t problemSolver_;
          graph::GraphPtr_t graph_;
      }; // class Graph
    } // namespace impl
  } // namespace manipulation
} // namespace hpp

#endif // HPP_MANIPULATION_CORBA_GRAPH_IMPL_HH
