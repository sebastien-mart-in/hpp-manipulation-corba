// Copyright (C) 2014 CNRS-LAAS
// Author: Florent Lamiraux.
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
#include <hpp/util/exception.hh>

#include "graph.impl.hh"
#include "problem.impl.hh"
#include "robot.impl.hh"

namespace hpp {
namespace manipulation {
Server::Server(corbaServer::Server* server)
    : corbaServer::ServerPlugin(server),
      graphImpl_(NULL),
      problemImpl_(NULL),
      robotImpl_(NULL) {}

Server::~Server() {
  if (graphImpl_) delete graphImpl_;
  if (problemImpl_) delete problemImpl_;
  if (robotImpl_) delete robotImpl_;
}

std::string Server::name() const { return "manipulation"; }

/// Start corba server
void Server::startCorbaServer(const std::string& contextId,
                              const std::string& contextKind) {
  initializeTplServer(graphImpl_, contextId, contextKind, name(), "graph");
  initializeTplServer(problemImpl_, contextId, contextKind, name(), "problem");
  initializeTplServer(robotImpl_, contextId, contextKind, name(), "robot");

  graphImpl_->implementation().setServer(this);
  problemImpl_->implementation().setServer(this);
  robotImpl_->implementation().setServer(this);
}

ProblemSolverPtr_t Server::problemSolver() {
  ProblemSolverPtr_t psm =
      dynamic_cast<ProblemSolverPtr_t>(problemSolverMap_->selected());
  if (psm == NULL)
    throw std::logic_error("ProblemSolver is not a manipulation problem");
  return psm;
}

::CORBA::Object_ptr Server::servant(const std::string& name) const {
  if (name == "graph") return graphImpl_->implementation()._this();
  if (name == "problem") return problemImpl_->implementation()._this();
  if (name == "robot") return robotImpl_->implementation()._this();
  throw std::invalid_argument("No servant " + name);
}
}  // namespace manipulation
}  // namespace hpp

HPP_CORBASERVER_DEFINE_PLUGIN(hpp::manipulation::Server)
