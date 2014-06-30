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
#include <hpp/util/debug.hh>
#include <hpp/core/config-projector.hh>
#include <hpp/model/gripper.hh>
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
				 const char* gripperName,
				 const char* handleName)
	throw (hpp::Error)
      {
	RobotPtr_t robot = problemSolver_->robot ();
	if (!robot) {
	  throw Error ("You should build a composite robot before trying to"
		       " define constraints.");
	}
	try {
	  const GripperPtr_t gripper = robot->gripper (gripperName);
	  const HandlePtr_t& handle = robot->handle (handleName);
	  DifferentiableFunctionPtr_t constraint =
	    handle->createGrasp (gripper);
	  problemSolver_->addNumericalConstraint (graspName, constraint);
          problemSolver_->addGrasp(graspName, gripper, handle);
	} catch (const std::exception& exc) {
	  throw Error (exc.what ());
	}
      }

      void Problem::resetConstraints ()	throw (hpp::Error)
      {
	try {
	  problemSolver_->resetConstraints ();
          ProblemSolver::GraspsMap_t grasps = problemSolver_->grasps();
          for (ProblemSolver::GraspsMap_t::const_iterator itGrasp = grasps.begin() ;  
                 itGrasp != grasps.end() ; itGrasp++) {
            ProblemSolver::GraspPtr_t grasp = itGrasp->second;
            GripperPtr_t gripper = grasp->first;
            HandlePtr_t handle = grasp->second;
            JointPtr_t joint = handle->joint();
            model::JointVector_t joints = gripper->getDisabledCollisions();
            for (model::JointVector_t::iterator itJoint = joints.begin() ;
                 itJoint != joints.end() ; itJoint++ ) {
              problemSolver_->robot()->addCollisionPairs(joint, *itJoint,
                                           hpp::model::COLLISION);
              problemSolver_->robot()->addCollisionPairs(joint, *itJoint,
                                           hpp::model::DISTANCE);
            }
          }
	} catch (const std::exception& exc) {
	  throw hpp::Error (exc.what ());
	}
      }

      // ---------------------------------------------------------------

      void Problem::setNumericalConstraints
      (const char* constraintName, const Names_t& constraintNames)
	throw (Error)
      {
	using core::ConstraintSetPtr_t;
	using core::ConfigProjector;
	using core::ConfigProjectorPtr_t;
	try {
	  const ConstraintSetPtr_t& constraints
	    (problemSolver_->constraints ());
	  const DevicePtr_t& robot (problemSolver_->robot ());
	  if (!robot) {
	    throw Error ("You should set the robot before defining"
			 " constraints.");
	  }
	  ConfigProjectorPtr_t  configProjector =
	    constraints->configProjector ();
	  if (!configProjector) {
	    configProjector = ConfigProjector::create
	      (robot, constraintName, problemSolver_->errorThreshold (),
	       problemSolver_->maxIterations ());
	    constraints->addConstraint (configProjector);
	  }
	  for (CORBA::ULong i=0; i<constraintNames.length (); ++i) {
	    std::string name (constraintNames [i]);
	    configProjector->addConstraint (problemSolver_->numericalConstraint
					    (name));
            if ( problemSolver_->grasp(name) ) {
              GripperPtr_t gripper = problemSolver_->grasp(name)->first;
              HandlePtr_t handle = problemSolver_->grasp(name)->second;
              JointPtr_t joint1 = handle->joint();
              model::JointVector_t joints = gripper->getDisabledCollisions();
              for (model::JointVector_t::iterator itJoint = joints.begin() ;
                 itJoint != joints.end() ; itJoint++ ) {
                problemSolver_->robot()->removeCollisionPairs(joint1, *itJoint,
                                           hpp::model::COLLISION);
                problemSolver_->robot()->removeCollisionPairs(joint1, *itJoint,
                                           hpp::model::DISTANCE);
              }
            }
	  }
	} catch (const std::exception& exc) {
	  throw Error (exc.what ());
	}
      }

    } // namespace impl
  } // namespace manipulation
} // namespace hpp
