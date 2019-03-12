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
      bool mThd = parent()->multiThread();
      graphImpl_   = new corba::Server <impl::Graph>   (0, NULL, mThd, "child");
      problemImpl_ = new corba::Server <impl::Problem> (0, NULL, mThd, "child");
      robotImpl_   = new corba::Server <impl::Robot>   (0, NULL, mThd, "child");

      graphImpl_  ->implementation ().setServer (this);
      problemImpl_->implementation ().setServer (this);
      robotImpl_  ->implementation ().setServer (this);

      if (graphImpl_->startCorbaServer(contextId, contextKind,
				       "manipulation", "graph") != 0) {
	HPP_THROW_EXCEPTION (hpp::Exception,
			     "Failed to start corba graph server.");
      }
      if (robotImpl_->startCorbaServer(contextId, contextKind,
				       "manipulation", "robot") != 0) {
	HPP_THROW_EXCEPTION (hpp::Exception,
			     "Failed to start corba robot server.");
      }
      if (problemImpl_->startCorbaServer(contextId, contextKind,
					 "manipulation", "problem") != 0) {
	HPP_THROW_EXCEPTION (hpp::Exception,
			     "Failed to start corba problem server.");
      }
    }

    ProblemSolverPtr_t Server::problemSolver ()
        throw (std::logic_error)
    {
      ProblemSolverPtr_t psm = dynamic_cast <ProblemSolverPtr_t>
        (problemSolverMap_->selected());
      if (psm == NULL)
        throw std::logic_error ("ProblemSolver is not a manipulation problem");
      return psm;
    }
  } // namespace manipulation
} // namespace hpp

HPP_CORBASERVER_DEFINE_PLUGIN(hpp::manipulation::Server)
