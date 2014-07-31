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

#include <hpp/util/debug.hh>
#include <hpp/core/locked-dof.hh>
#include <hpp/core/constraint-set.hh>
#include <hpp/core/config-projector.hh>
#include <hpp/model/gripper.hh>
#include <hpp/manipulation/robot.hh>
#include <hpp/manipulation/object.hh>
#include <hpp/manipulation/manipulation-planner.hh>
#include "problem.impl.hh"

namespace hpp {
  namespace manipulation {
    namespace impl {
      static ConfigurationPtr_t floatSeqToConfig
      (hpp::manipulation::ProblemSolverPtr_t problemSolver,
       const hpp::floatSeq& dofArray)
      {
	size_type configDim = (size_type)dofArray.length();
	ConfigurationPtr_t config (new Configuration_t (configDim));

	// Get robot in hppPlanner object.
	DevicePtr_t robot = problemSolver->robot ();

	// Compare size of input array with number of degrees of freedom of
	// robot.
	if (configDim != robot->configSize ()) {
	  hppDout (error, "robot nb dof=" << configDim <<
		   " is different from config size=" << robot->configSize());
	  throw std::runtime_error
	    ("robot nb dof is different from config size");
	}

	// Fill dof vector with dof array.
	for (size_type iDof=0; iDof < configDim; ++iDof) {
	  (*config) [iDof] = dofArray [iDof];
	}
	return config;
      }

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
          problemSolver_->addGrasp(constraint, gripper, handle);
	} catch (const std::exception& exc) {
	  throw Error (exc.what ());
	}
      }

      void Problem::createLockedDofConstraint (const char* lockedDofName,
                             const char* jointName, Double value,
			     UShort rankInConfiguration, UShort rankInVelocity)
	throw (hpp::Error)
      {
	try {
	  // Get robot in hppPlanner object.
	  RobotPtr_t robot = problemSolver_->robot ();
	  JointPtr_t joint = robot->getJointByName (jointName);

	  LockedDofPtr_t lockedDof (LockedDof::create (lockedDofName, joint,
						       value,
						       rankInConfiguration,
						       rankInVelocity));
          problemSolver_->addLockedDofConstraint (lockedDofName, lockedDof);
	} catch (const std::exception& exc) {
	  throw hpp::Error (exc.what ());
	}
      }

      void Problem::isLockedDofParametric (const char* constraintName,
          CORBA::Boolean value)
        throw (hpp::Error)
      {
        LockedDofPtr_t l =
          problemSolver_->lockedDofConstraint (constraintName);
        if (!l)
          throw hpp::Error ("The LockedDof constraint could not be found.");
        l->isParametric (value);
      }

      void Problem::extend (const hpp::floatSeq& q_near,
          const hpp::floatSeq& q_rand,
          hpp::floatSeq_out output)
      {
	ConfigurationPtr_t cfg_near = floatSeqToConfig (problemSolver_, q_near);
	ConfigurationPtr_t cfg_rand = floatSeqToConfig (problemSolver_, q_rand);
        Configuration_t cfg_new;
	try {
          ManipulationPlannerPtr_t planner =
            HPP_DYNAMIC_PTR_CAST(ManipulationPlanner, problemSolver_->pathPlanner ());
          if (!planner)
            throw hpp::Error ("The planner must be a ManipulationPlanner");
          core::PathPtr_t path;
          planner->extend (cfg_near, cfg_rand, path);
          cfg_new = (*path) (path->length());
	} catch (const std::exception& exc) {
	  throw hpp::Error (exc.what ());
	}
	ULong size = (ULong) cfg_new.size ();
	hpp::floatSeq* q_ptr = new hpp::floatSeq ();
	q_ptr->length (size);

	for (std::size_t i=0; i<size; ++i) {
	  (*q_ptr) [i] = cfg_new [i];
	}
	output = q_ptr;
      }
    } // namespace impl
  } // namespace manipulation
} // namespace hpp
