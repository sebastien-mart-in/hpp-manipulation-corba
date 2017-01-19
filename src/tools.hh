// Copyright (c) 2016 CNRS
// Author: Joseph Mirabel
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
