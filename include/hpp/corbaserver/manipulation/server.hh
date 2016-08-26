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

#ifndef HPP_MANIPULATION_CORBA_SERVER_HH
# define HPP_MANIPULATION_CORBA_SERVER_HH

# include <stdexcept>

# include <hpp/corba/template/server.hh>

# include <hpp/corbaserver/problem-solver-map.hh>

# include <hpp/corbaserver/manipulation/fwd.hh>
# include <hpp/corbaserver/manipulation/config.hh>

namespace hpp {
  namespace manipulation {
    namespace impl {
      class Graph;
      class Problem;
      class Robot;
    }
    class HPP_MANIPULATION_CORBA_DLLAPI Server
    {
    public:
      Server (int argc, const char* argv[], bool multiThread = false,
	      const std::string& poaName = "child");
      ~Server ();

      /// Set planner that will be controlled by server
      void setProblemSolverMap (corbaServer::ProblemSolverMapPtr_t psMap)
      {
        problemSolverMap_ = psMap;
      }

      /// Start corba server

      /// Call hpp::corba::Server <impl::Problem>::startCorbaServer
      void startCorbaServer(const std::string& contextId,
			    const std::string& contextKind,
			    const std::string& objectId);

      ProblemSolverPtr_t problemSolver () throw (std::logic_error);

      corbaServer::ProblemSolverMapPtr_t problemSolverMap ();

    private:
      corba::Server <impl::Graph>* graphImpl_;
      corba::Server <impl::Problem>* problemImpl_;
      corba::Server <impl::Robot>* robotImpl_;

      corbaServer::ProblemSolverMapPtr_t problemSolverMap_;
    }; // class Server
  } // namespace manipulation
} // namespace hpp

#endif // HPP_MANIPULATION_CORBA_SERVER_HH
