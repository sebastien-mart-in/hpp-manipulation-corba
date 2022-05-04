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

#ifndef HPP_MANIPULATION_CORBA_SERVER_HH
#define HPP_MANIPULATION_CORBA_SERVER_HH

#include <hpp/corba/template/server.hh>
#include <hpp/corbaserver/manipulation/config.hh>
#include <hpp/corbaserver/manipulation/fwd.hh>
#include <hpp/corbaserver/server-plugin.hh>
#include <stdexcept>

namespace hpp {
namespace manipulation {
namespace impl {
class Graph;
class Problem;
class Robot;
}  // namespace impl
class HPP_MANIPULATION_CORBA_DLLAPI Server : public corbaServer::ServerPlugin {
 public:
  Server(corbaServer::Server* parent);

  ~Server();

  /// Start corba server
  /// Call hpp::corba::Server <impl::Problem>::startCorbaServer
  void startCorbaServer(const std::string& contextId,
                        const std::string& contextKind);

  ::CORBA::Object_ptr servant(const std::string& name) const;

  std::string name() const;

  ProblemSolverPtr_t problemSolver();

 private:
  corba::Server<impl::Graph>* graphImpl_;
  corba::Server<impl::Problem>* problemImpl_;
  corba::Server<impl::Robot>* robotImpl_;
};  // class Server
}  // namespace manipulation
}  // namespace hpp

#endif  // HPP_MANIPULATION_CORBA_SERVER_HH
