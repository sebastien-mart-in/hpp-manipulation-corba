// Copyright (c) 2012 CNRS
// Author: Florent Lamiraux
//
// This file is part of hpp-manipulation-corba.
// hpp-manipulation-corba is free software: you can redistribute it
// and/or modify it under the terms of the GNU Lesser General Public
// License as published by the Free Software Foundation, either version
// 3 of the License, or (at your option) any later version.
//
// hpp-manipulation-corba is distributed in the hope that it will be
// useful, but WITHOUT ANY WARRANTY; without even the implied warranty
// of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
// General Lesser Public License for more details.  You should have
// received a copy of the GNU Lesser General Public License along with
// hpp-manipulation-corba.  If not, see
// <http://www.gnu.org/licenses/>.

#include <hpp/corbaserver/server.hh>
#include <hpp/manipulation/package-config.hh>
#include <hpp/corbaserver/manipulation/server.hh>
#include <hpp/manipulation/problem-solver.hh>

typedef hpp::corbaServer::Server CorbaServer;
typedef hpp::manipulation::Server ManipulationServer;
typedef hpp::manipulation::ProblemSolver ProblemSolver;
typedef hpp::manipulation::ProblemSolverPtr_t ProblemSolverPtr_t;

int main (int argc, char* argv [])
{
  if (argc > 0) std::cerr << argv[0];
  else          std::cerr << "hpp-manipulation-server";
  std::cerr << " is provided for backward compatibility.\n"
    "You can now use hppcorbaserver and add the following lines to your Python script:\n"
    "from hpp.corbaserver import loadServerPlugin\n"
    "loadServerPlugin (\"corbaserver\", \"manipulation-corba.so\")\n"
    "# eventually\n"
    "# loadServerPlugin (\"corbaserver\", \"wholebody-step-corba.so\")\n"
    << std::endl;

  ProblemSolverPtr_t problemSolver = new ProblemSolver();

  CorbaServer corbaServer (problemSolver, argc,
			   const_cast<const char**> (argv), true);
  corbaServer.startCorbaServer ();

  corbaServer.loadPlugin (corbaServer.mainContextId(), "manipulation-corba.so");
  corbaServer.processRequest(true);
}
