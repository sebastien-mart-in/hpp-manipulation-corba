// Copyright (C) 2014 CNRS-LAAS
// Author: Florent Lamiraux.
//
// This file is part of the hpp-manipulation-corba.
//
// hpp-manipulation-corba is free software: you can redistribute
// it and/or modify it under the terms of the GNU Lesser General
// Public License as published by the Free Software Foundation, either
// version 3 of the License, or (at your option) any later version.
//
// hpp-manipulation-corba is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU Lesser General Public License for more details.
//
// You should have received a copy of the GNU Lesser General Public
// License along with hpp-manipulation-corba.  If not, see
// <http://www.gnu.org/licenses/>.

#include <hpp/corbaserver/manipulation/server.hh>

#include <hpp/util/exception.hh>
#include <hpp/corbaserver/server.hh>
#include "graph.impl.hh"
#include "problem.impl.hh"
#include "robot.impl.hh"

namespace hpp {
  namespace manipulation {
    Server::Server (corbaServer::Server* server)
      : corbaServer::ServerPlugin (server),
      graphImpl_   (NULL),
      problemImpl_ (NULL),
      robotImpl_   (NULL)
    {}

    Server::~Server ()
    {
      if (graphImpl_  ) delete graphImpl_;
      if (problemImpl_) delete problemImpl_;
      if (robotImpl_  ) delete robotImpl_;
    }

    std::string Server::name () const
    {
      return "manipulation";
    }

    /// Start corba server
    void Server::startCorbaServer(const std::string& contextId,
				  const std::string& contextKind)
    {
      initializeTplServer(graphImpl_  , contextId, contextKind, name(), "graph");
      initializeTplServer(problemImpl_, contextId, contextKind, name(), "problem");
      initializeTplServer(robotImpl_  , contextId, contextKind, name(), "robot");

      graphImpl_  ->implementation ().setServer (this);
      problemImpl_->implementation ().setServer (this);
      robotImpl_  ->implementation ().setServer (this);
    }

    ProblemSolverPtr_t Server::problemSolver ()
    {
      ProblemSolverPtr_t psm = dynamic_cast <ProblemSolverPtr_t>
        (problemSolverMap_->selected());
      if (psm == NULL)
        throw std::logic_error ("ProblemSolver is not a manipulation problem");
      return psm;
    }

    ::CORBA::Object_ptr Server::servant(const std::string& name) const
    {
      if (name == "graph"  ) return graphImpl_  ->implementation()._this();
      if (name == "problem") return problemImpl_->implementation()._this();
      if (name == "robot"  ) return robotImpl_  ->implementation()._this();
      throw std::invalid_argument ("No servant " + name);
    }
  } // namespace manipulation
} // namespace hpp

HPP_CORBASERVER_DEFINE_PLUGIN(hpp::manipulation::Server)
