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
#include <hpp/constraints/static-stability.hh>
#include <hpp/manipulation/robot.hh>
#include <hpp/manipulation/object.hh>
#include <hpp/manipulation/manipulation-planner.hh>
#include <hpp/manipulation/graph/node.hh>
#include <hpp/manipulation/graph/edge.hh>
#include <hpp/manipulation/axial-handle.hh>
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
	  DifferentiableFunctionPtr_t complement =
	    handle->createGraspComplement (gripper);
	  problemSolver_->addNumericalConstraint (graspName, constraint);
	  problemSolver_->addNumericalConstraint (std::string(graspName) + "/complement", complement);
          problemSolver_->addGrasp(constraint, gripper, handle);
	} catch (const std::exception& exc) {
	  throw Error (exc.what ());
	}
      }

      void Problem::createPreGrasp (const char* graspName,
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
          using hpp::core::EquationType;
	  const GripperPtr_t gripper = robot->gripper (gripperName);
	  const HandlePtr_t& handle = robot->handle (handleName);
          std::string name (graspName);
	  DifferentiableFunctionPtr_t constraint =
	    handle->createPreGrasp (gripper);
	  DifferentiableFunctionPtr_t ineq_positive =
	    handle->createPreGraspComplement (gripper, 0.05);
	  DifferentiableFunctionPtr_t ineq_negative =
	    handle->createPreGraspComplement (gripper, 0.1);
	  problemSolver_->addNumericalConstraint (graspName, constraint);
	  problemSolver_->addNumericalConstraint (name + "/ineq_0", ineq_positive);
	  problemSolver_->addNumericalConstraint (name + "/ineq_0.1", ineq_negative);
          problemSolver_->addInequalityVector (name + "/ineq_0", EquationType::VectorOfTypes (1, EquationType::Inferior));
          problemSolver_->addInequalityVector (name + "/ineq_0.1", EquationType::VectorOfTypes (1, EquationType::Superior));
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

      void Problem::createPlacementConstraint (const char* placName,
          const char* objectName,
          const char* objectJointName,
          const char* objectTriangleName,
          const char* envContactName)
        throw (hpp::Error)
      {
	try {
	  // Get robot in hppPlanner object.
	  RobotPtr_t robot = problemSolver_->robot ();
	  JointPtr_t joint = robot->getJointByName (objectJointName);

	  ObjectPtr_t object = problemSolver_->object (objectName);

          using constraints::StaticStabilityGravity;
          using constraints::StaticStabilityGravityPtr_t;
          StaticStabilityGravityPtr_t c = StaticStabilityGravity::create (robot, joint);

          TriangleList l = object->contactTriangles (objectTriangleName);
          for (TriangleList::const_iterator it = l.begin (); it != l.end(); it++)
            c->addObjectTriangle (*it);
          l = problemSolver_->contactTriangles (envContactName);
          for (TriangleList::const_iterator it = l.begin (); it != l.end(); it++)
            c->addFloorTriangle (*it);

          problemSolver_->addNumericalConstraint (placName, c);
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

      bool Problem::applyConstraints (hpp::ID id,
          const hpp::floatSeq& input,
          hpp::floatSeq_out output,
          double& residualError)
        throw (hpp::Error)
      {
        /// First get the constraint.
        ConstraintSetPtr_t constraint;
        try {
          graph::GraphComponentPtr_t comp = graph::GraphComponent::get ((size_t)id).lock ();
          graph::EdgePtr_t edge = HPP_DYNAMIC_PTR_CAST(graph::Edge, comp);
          graph::NodePtr_t node = HPP_DYNAMIC_PTR_CAST(graph::Node, comp);
          if (edge)
            constraint = problemSolver_->constraintGraph ()->configConstraint (edge);
          else if (node)
            constraint = problemSolver_->constraintGraph ()->configConstraint (node);
          else {
            std::stringstream ss;
            ss << "ID " << id << " is neither an edge nor a node";
            std::string errmsg = ss.str();
            throw Error (errmsg.c_str());
          }
        } catch (std::exception& e ) {
          throw Error (e.what());
        }

	bool success = false;
	ConfigurationPtr_t config = floatSeqToConfig (problemSolver_, input);
	try {
	  success = constraint->apply (*config);
	  if (hpp::core::ConfigProjectorPtr_t configProjector =
	      constraint ->configProjector ()) {
	    residualError = configProjector->residualError ();
	  }
	} catch (const std::exception& exc) {
	  throw hpp::Error (exc.what ());
	}
	ULong size = (ULong) config->size ();
	hpp::floatSeq* q_ptr = new hpp::floatSeq ();
	q_ptr->length (size);

	for (std::size_t i=0; i<size; ++i) {
	  (*q_ptr) [i] = (*config) [i];
	}
	output = q_ptr;
	return success;
      }

      bool Problem::applyConstraintsWithOffset (hpp::ID IDedge,
          const hpp::floatSeq& qnear,
          const hpp::floatSeq& input,
          hpp::floatSeq_out output,
          double& residualError)
        throw (hpp::Error)
      {
        /// First get the constraint.
        ConstraintSetPtr_t constraint;
        try {
          graph::EdgePtr_t edge = HPP_DYNAMIC_PTR_CAST(graph::Edge,
                  graph::GraphComponent::get((size_t)IDedge).lock ());
          if (!edge) {
            std::stringstream ss;
            ss << "ID " << IDedge << " is not an edge";
            std::string errmsg = ss.str();
            throw Error (errmsg.c_str());
          }
          constraint = problemSolver_->constraintGraph ()->configConstraint (edge);
          ConfigurationPtr_t qoffset = floatSeqToConfig (problemSolver_, qnear);
          constraint->offsetFromConfig (*qoffset);
        } catch (std::exception& e ) {
          throw Error (e.what());
        }

	bool success = false;
	ConfigurationPtr_t config = floatSeqToConfig (problemSolver_, input);
	try {
	  success = constraint->apply (*config);
	  if (hpp::core::ConfigProjectorPtr_t configProjector =
	      constraint ->configProjector ()) {
	    residualError = configProjector->residualError ();
	  }
	} catch (const std::exception& exc) {
	  throw hpp::Error (exc.what ());
	}
	ULong size = (ULong) config->size ();
	hpp::floatSeq* q_ptr = new hpp::floatSeq ();
	q_ptr->length (size);

	for (std::size_t i=0; i<size; ++i) {
	  (*q_ptr) [i] = (*config) [i];
	}
	output = q_ptr;
	return success;
      }
    } // namespace impl
  } // namespace manipulation
} // namespace hpp
