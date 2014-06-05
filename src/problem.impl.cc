// Copyright (c) 2014 CNRS
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

#include <hpp/core/constraint-set.hh>
#include <hpp/core/config-projector.hh>
#include <hpp/manipulation/gripper.hh>
#include <hpp/manipulation/robot.hh>
#include <hpp/manipulation/object.hh>
#include "problem.impl.hh"

namespace hpp {
  namespace manipulation {
    namespace impl {
      Problem::Problem () : problemSolver_ (0x0)
      {
      }

      void Problem::createGrasp (const char* graspName,
				 const char* jointName,
				 const char* handleName,
				 const CORBA::Double* handlePositioninJoint)
	throw (hpp::Error)
      {
	RobotPtr_t robot = problemSolver_->robot ();
	if (!robot) {
	  throw Error ("You should build a composite robot before trying to"
		       " define constraints.");
	}
	try {
	  JointPtr_t joint = robot->getJointByName (jointName);
	  const HandlePtr_t& handle = robot->handle (handleName);
	  fcl::Quaternion3f q (handlePositioninJoint [3],
			       handlePositioninJoint [4],
			       handlePositioninJoint [5],
			       handlePositioninJoint [6]);
	  fcl::Vec3f v (handlePositioninJoint [0], handlePositioninJoint [1],
			handlePositioninJoint [2]);
	  GripperPtr_t gripper = Gripper::create (joint, Transform3f (q, v));
	  DifferentiableFunctionPtr_t constraint =
	    handle->createGrasp ( gripper);
	  problemSolver_->addNumericalConstraint (graspName, constraint);
	} catch (const std::exception& exc) {
	  throw Error (exc.what ());
	}
      }
    } // namespace impl
  } // namespace manipulation
} // namespace hpp
