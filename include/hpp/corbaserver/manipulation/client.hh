// Copyright (C) 2015 by Joseph Mirabel
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
//
// This software is provided "as is" without warranty of any kind,
// either expressed or implied, including but not limited to the
// implied warranties of fitness for a particular purpose.
//
// See the COPYING file for more information.

#ifndef HPP_MANIPULATION_CORBA_CLIENT_HH
#define HPP_MANIPULATION_CORBA_CLIENT_HH

#include <omniORB4/CORBA.h>

#include <hpp/corbaserver/client.hh>
#include <hpp/corbaserver/manipulation/config.hh>
#include <hpp/corbaserver/manipulation/graph-idl.hh>
#include <hpp/corbaserver/manipulation/problem-idl.hh>
#include <hpp/corbaserver/manipulation/robot-idl.hh>
#include <hpp/corbaserver/obstacle-idl.hh>

namespace hpp {
namespace corbaServer {
namespace manipulation {
class HPP_MANIPULATION_CORBA_DLLAPI Client : public ClientBase {
 public:
  Client(int argc, char* argv[]);

  ~Client();

  /// \param iiop base address
  /// \param context the hpp context name (passed to the server)
  void connect(const char* iiop = "corbaloc:iiop:",
               const char* context = "corbaserver");

  hpp::corbaserver::manipulation::Robot_var& robot() { return robot_; }

  hpp::corbaserver::manipulation::Problem_var& problem() { return problem_; }

  hpp::corbaserver::manipulation::Graph_var& graph() { return graph_; }

 private:
  hpp::corbaserver::manipulation::Robot_var robot_;
  hpp::corbaserver::manipulation::Problem_var problem_;
  hpp::corbaserver::manipulation::Graph_var graph_;
};
}  // end of namespace manipulation.
}  // end of namespace corbaServer.
}  // end of namespace hpp.

#endif  // HPP_MANIPULATION_CORBA_CLIENT_HH
