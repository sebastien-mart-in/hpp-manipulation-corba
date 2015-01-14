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
#include <hpp/core/comparison-type.hh>
#include <hpp/core/locked-joint.hh>
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

      core::ComparisonTypePtr_t stringToComparisonType (const std::string& s, const value_type& thr = 0) {
        if (s.compare ("Equality") == 0)
          return core::Equality::create ();
        if (s.compare ("EqualToZero") == 0)
          return core::EqualToZero::create ();
        if (s.compare ("SuperiorIneq") == 0)
          return core::SuperiorIneq::create (thr);
        if (s.compare ("InferiorIneq") == 0)
          return core::InferiorIneq::create (thr);
        throw Error ((s + std::string (" is not a ComparisonType")).c_str ());
      }

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

      static vector_t floatSeqToVector (const hpp::floatSeq& dofArray)
      {
	// Fill dof vector with dof array.
	vector_t result (dofArray.length ());
	for (size_type iDof=0; iDof < result.size (); ++iDof) {
	  result [iDof] = dofArray [iDof];
	}
	return result;
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
          using hpp::core::DoubleInequality;
	  const GripperPtr_t gripper = robot->gripper (gripperName);
	  const HandlePtr_t& handle = robot->handle (handleName);
          std::string name (graspName);
	  DifferentiableFunctionPtr_t constraint =
	    handle->createPreGrasp (gripper);
	  DifferentiableFunctionPtr_t ineq_positive =
	    handle->createPreGraspComplement (gripper, 0.023);
	  problemSolver_->addNumericalConstraint (graspName, constraint);
	  problemSolver_->addNumericalConstraint (name + "/0_f_0.05", ineq_positive);
          problemSolver_->comparisonType (name + "/0_f_0.05", DoubleInequality::create (0.05));
          problemSolver_->addGrasp(constraint, gripper, handle);
	} catch (const std::exception& exc) {
	  throw Error (exc.what ());
	}
      }

      void Problem::createLockedJoint
      (const char* lockedJointName, const char* jointName,
       const hpp::floatSeq& value, const char* comparisonType)
	throw (hpp::Error)
      {
	try {
	  // Get robot in hppPlanner object.
	  RobotPtr_t robot = problemSolver_->robot ();
	  JointPtr_t joint = robot->getJointByName (jointName);
	  vector_t config = floatSeqToVector (value);

          core::ComparisonTypePtr_t compType = stringToComparisonType (comparisonType);
          LockedJointPtr_t lockedJoint (LockedJoint::create (joint, config, compType));
          problemSolver_->addLockedJoint (lockedJointName, lockedJoint);
	} catch (const std::exception& exc) {
	  throw hpp::Error (exc.what ());
	}
      }

      Names_t* Problem::getEnvironmentContactNames ()
        throw (hpp::Error)
      {
        try {
	  const TriangleMap& m = problemSolver_->contactTriangles ();

	  char** nameList = Names_t::allocbuf(m.size ());
	  Names_t *jointNames = new Names_t (m.size(), m.size(), nameList);

	  std::size_t rank = 0;
          for (TriangleMap::const_iterator it = m.begin ();
              it != m.end (); it++) {
            nameList[rank] = (char*) malloc (sizeof(char)*(it->first.length ()+1));
            strcpy (nameList [rank], it->first.c_str ());
            rank++;
          }
	  return jointNames;
	} catch (const std::exception& exc) {
	  throw hpp::Error (exc.what ());
        }
      }

      Names_t* Problem::getObjectContactNames (const char* objectName)
        throw (hpp::Error)
      {
        try {
	  ObjectPtr_t o = problemSolver_->object (std::string (objectName));
          if (!o)
            throw Error ("The object does not exists.");
	  const TriangleMap& m = o->contactTriangles ();

	  char** nameList = Names_t::allocbuf(m.size ());
	  Names_t *jointNames = new Names_t (m.size(), m.size(), nameList);

	  std::size_t rank = 0;
          for (TriangleMap::const_iterator it = m.begin ();
              it != m.end (); it++) {
            nameList[rank] = (char*) malloc (sizeof(char)*(it->first.length ()+1));
            strcpy (nameList [rank], it->first.c_str ());
            rank++;
          }
	  return jointNames;
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
          if (l.empty ())
            throw Error ((std::string ("The object has no triangle named ")
                        + objectTriangleName + std::string(".")).c_str ());
          for (TriangleList::const_iterator it = l.begin (); it != l.end(); it++)
            c->addObjectTriangle (*it);
          l = problemSolver_->contactTriangles (envContactName);
          if (l.empty ())
            throw Error ((std::string ("The environment has no triangle named ")
                        + envContactName + std::string(".")).c_str ());
          for (TriangleList::const_iterator it = l.begin (); it != l.end(); it++)
            c->addFloorTriangle (*it);

          problemSolver_->addNumericalConstraint (placName, c);
	} catch (const std::exception& exc) {
	  throw hpp::Error (exc.what ());
	}
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
          if (edge) {
            constraint =
	      problemSolver_->constraintGraph ()->configConstraint (edge);
            RobotPtr_t robot = problemSolver_->robot ();
            if (!robot) throw Error ("You must have a robot to do that.");
	    if (core::ConfigProjectorPtr_t cp =
		constraint->configProjector ()) {
	      cp->rightHandSideFromConfig (robot->currentConfiguration());
	    }
          } else if (node)
            constraint =
	      problemSolver_->constraintGraph ()->configConstraint (node);
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
          constraint =
	    problemSolver_->constraintGraph ()->configConstraint (edge);
          ConfigurationPtr_t qoffset = floatSeqToConfig (problemSolver_, qnear);
	  if (core::ConfigProjectorPtr_t cp = constraint->configProjector ()) {
	    cp->rightHandSideFromConfig (*qoffset);
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
    } // namespace impl
  } // namespace manipulation
} // namespace hpp
