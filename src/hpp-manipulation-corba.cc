// Copyright (c) 2012 CNRS
// Author: Florent Lamiraux
//

// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are
// met:
//
// 1. Redistributions of source code must retain the above copyright
//    notice, this list of conditions and the following disclaimer.
//
// 2. Redistributions in binary form must reproduce the above copyright
// notice, this list of conditions and the following disclaimer in the
// documentation and/or other materials provided with the distribution.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
// "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
// LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
// A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
// HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
// SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
// LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
// DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
// THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
// (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
// OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH
// DAMAGE.

#include <hpp/corbaserver/manipulation/server.hh>
#include <hpp/corbaserver/server.hh>
#include <hpp/manipulation/package-config.hh>
#include <hpp/manipulation/problem-solver.hh>

typedef hpp::corbaServer::Server CorbaServer;
typedef hpp::manipulation::Server ManipulationServer;
typedef hpp::manipulation::ProblemSolver ProblemSolver;
typedef hpp::manipulation::ProblemSolverPtr_t ProblemSolverPtr_t;

int main(int argc, char* argv[]) {
  if (argc > 0)
    std::cerr << argv[0];
  else
    std::cerr << "hpp-manipulation-server";
  std::cerr << " is provided for backward compatibility.\n"
               "You can now use hppcorbaserver and add the following lines to "
               "your Python script:\n"
               "from hpp.corbaserver import loadServerPlugin\n"
               "loadServerPlugin (\"corbaserver\", \"manipulation-corba.so\")\n"
            << std::endl;

  ProblemSolverPtr_t problemSolver = new ProblemSolver();

  CorbaServer corbaServer(problemSolver, argc, const_cast<const char**>(argv),
                          true);
  corbaServer.startCorbaServer();

  corbaServer.loadPlugin(corbaServer.mainContextId(), "manipulation-corba.so");
  corbaServer.processRequest(true);
}
