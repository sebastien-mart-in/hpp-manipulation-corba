// Copyright (c) 2016 CNRS
// Author: Joseph Mirabel
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

#ifndef HPP_MANIPULATION_CORBA_TOOLS_HH
# define HPP_MANIPULATION_CORBA_TOOLS_HH

# include <pinocchio/spatial/se3.hpp>

# include <hpp/corbaserver/conversions.hh>

# include <hpp/manipulation/problem-solver.hh>

namespace hpp {
  using corbaServer::floatSeqToVector;
  using corbaServer::vectorToFloatSeq;
  using corbaServer::toIntSeq;
  using corbaServer::matrixToIntSeqSeq;
  using corbaServer::toNames_t;
  using corbaServer::c_str;
  using pinocchio::Transform3f;
  using manipulation::DevicePtr_t;
  using manipulation::ProblemSolverPtr_t;

  inline std::vector<std::string> toStringVector (const Names_t& names) {
    typedef std::vector<std::string> Out_t;
    return corbaServer::toStrings<Out_t> (names);
  }

  inline std::list<std::string> toStringList (const Names_t& names) {
    typedef std::list<std::string> Out_t;
    return corbaServer::toStrings<Out_t> (names);
  }

  inline void Transform3fTohppTransform (const Transform3f& transform, CORBA::Double* config)
  {
    corbaServer::toHppTransform (transform, config);
  }

  inline void hppTransformToTransform3f (const CORBA::Double* inConfig, Transform3f& transform)
  {
    corbaServer::toTransform3f (inConfig, transform);
  }

  DevicePtr_t getRobotOrThrow (ProblemSolverPtr_t p);
} // namespace hpp

#endif // HPP_MANIPULATION_CORBA_TOOLS_HH
