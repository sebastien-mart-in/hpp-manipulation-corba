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
#include <hpp/core/config-projector.hh>
#include <hpp/core/path-projector.hh>
#include <hpp/core/path-vector.hh>
#include <hpp/model/gripper.hh>
#include <hpp/constraints/convex-shape-contact.hh>
#include <hpp/constraints/qp-static-stability.hh>
#include <hpp/manipulation/device.hh>
#include <hpp/manipulation/problem.hh>
#include <hpp/manipulation/constraint-set.hh>
#include <hpp/manipulation/manipulation-planner.hh>
#include <hpp/manipulation/graph/node.hh>
#include <hpp/manipulation/graph/edge.hh>
#include <hpp/manipulation/axial-handle.hh>
#include "problem.impl.hh"

namespace hpp {
  namespace manipulation {
    namespace impl {
      namespace {
        core::ComparisonTypePtr_t stringToComparisonType (const std::string& s, const value_type& thr = 0) {
          // TODO: Comparison type DoubleInequality is omitted because the
          // constructor requires a width parameter.
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

        DevicePtr_t getRobotOrThrow (ProblemSolverPtr_t p)
        {
          DevicePtr_t robot = p->robot ();
          if (!robot) throw Error ("Robot not found.");
          return robot;
        }
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
	  (*config) [iDof] = dofArray [(ULong) iDof];
	}
	return config;
      }

      static vector_t floatSeqToVector (const hpp::floatSeq& dofArray)
      {
	// Fill dof vector with dof array.
	vector_t result (dofArray.length ());
	for (size_type iDof=0; iDof < result.size (); ++iDof) {
	  result [iDof] = dofArray [(ULong) iDof];
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
	DevicePtr_t robot = getRobotOrThrow (problemSolver_);
	hppDout (info, *robot);
	try {
          GripperPtr_t gripper = robot->get <GripperPtr_t> (gripperName);
	  if (!gripper) {
	    std::string msg (std::string ("No gripper with name ") +
			     std::string (gripperName) + std::string ("."));
	    throw std::runtime_error (msg.c_str ());
	  }
          HandlePtr_t handle = robot->get <HandlePtr_t> (handleName);
	  if (!handle) {
	    std::string msg (std::string ("No handle with name ") +
			     std::string (handleName) + std::string ("."));
	    throw std::runtime_error (msg.c_str ());
	  }
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
        DevicePtr_t robot = getRobotOrThrow (problemSolver_);
	try {
          using hpp::core::DoubleInequality;
          const GripperPtr_t gripper = robot->get <GripperPtr_t> (gripperName);
          const HandlePtr_t& handle = robot->get <HandlePtr_t> (handleName);
          std::string name (graspName);
          value_type c = handle->clearance () + gripper->clearance ();
          value_type width = 2*c * 1.01;
	  DifferentiableFunctionPtr_t constraint =
	    handle->createPreGrasp (gripper);
	  DifferentiableFunctionPtr_t ineq_positive =
	    handle->createPreGraspComplement (gripper, c / 2);
	  problemSolver_->addNumericalConstraint (name, constraint);
	  problemSolver_->addNumericalConstraint
            (name + "/double_ineq", ineq_positive);
          problemSolver_->comparisonType
            (name + "/double_ineq", DoubleInequality::create (width));
          problemSolver_->addGrasp(constraint, gripper, handle);
	} catch (const std::exception& exc) {
	  throw Error (exc.what ());
	}
      }

      void Problem::createLockedJoint
      (const char* lockedJointName, const char* jointName,
       const hpp::floatSeq& value)
	throw (hpp::Error)
      {
	try {
	  // Get robot in hppPlanner object.
          DevicePtr_t robot = getRobotOrThrow (problemSolver_);
	  JointPtr_t joint = robot->getJointByName (jointName);
	  vector_t config = floatSeqToVector (value);

          LockedJointPtr_t lockedJoint (LockedJoint::create (joint, config));
          problemSolver_->add <LockedJointPtr_t> (lockedJointName, lockedJoint);
	} catch (const std::exception& exc) {
	  throw hpp::Error (exc.what ());
	}
      }

      Names_t* Problem::getEnvironmentContactNames ()
        throw (hpp::Error)
      {
        try {
	  typedef Container <JointAndShapes_t>::ElementMap_t ShapeMap;
	  const ShapeMap& m = problemSolver_->getAll <JointAndShapes_t> ();

	  char** nameList = Names_t::allocbuf((ULong) m.size ());
	  Names_t *jointNames = new Names_t ((ULong) m.size(), (ULong) m.size(),
					     nameList);

	  std::size_t rank = 0;
          for (ShapeMap::const_iterator it = m.begin ();
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

      Names_t* Problem::getRobotContactNames ()
        throw (hpp::Error)
      {
        try {
	  typedef Container <JointAndShapes_t>::ElementMap_t ShapeMap;
          DevicePtr_t r = getRobotOrThrow (problemSolver_);
	  const ShapeMap& m = r->getAll <JointAndShapes_t> ();

	  char** nameList = Names_t::allocbuf((ULong) m.size ());
	  Names_t *jointNames = new Names_t ((ULong) m.size(), (ULong) m.size(),
					     nameList);

	  std::size_t rank = 0;
          for (ShapeMap::const_iterator it = m.begin ();
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
          const char* shapeName, const char* envContactName)
        throw (hpp::Error)
      {
	try {
	  // Get robot in hppPlanner object.
          DevicePtr_t robot = getRobotOrThrow (problemSolver_);

          using constraints::ConvexShape;
          using constraints::ConvexShapeContact;
          using constraints::ConvexShapeContactPtr_t;
          ConvexShapeContactPtr_t c = ConvexShapeContact::create (placName, robot);

          JointAndShapes_t l = robot->get <JointAndShapes_t> (shapeName);
          if (l.empty ()) throw Error ("Robot shapes not found.");
          for (JointAndShapes_t::const_iterator it = l.begin (); it != l.end(); it++)
            c->addObject (ConvexShape (it->second, it->first));
          if (l.empty ()) throw Error ("Environment shapes not found.");

	  // Search first robot triangles
          l = robot->get <JointAndShapes_t> (envContactName);
	  if (l.empty ()) {
	    // and then environment triangles.
            l = problemSolver_->get <JointAndShapes_t> (envContactName);
            if (l.empty ()) throw Error ("Environment triangles not found.");
	  }
          for (JointAndShapes_t::const_iterator it = l.begin (); it != l.end(); it++)
            c->addFloor (ConvexShape (it->second, it->first));

          problemSolver_->addNumericalConstraint (placName, c);
	} catch (const std::exception& exc) {
	  throw hpp::Error (exc.what ());
	}
      }

      void Problem::createQPStabilityConstraint (const char* placName,
          const Names_t& shapesName)
        throw (hpp::Error)
      {
	try {
	  // Get robot in hppPlanner object.
          DevicePtr_t robot = getRobotOrThrow (problemSolver_);

          using constraints::ConvexShape;
          using constraints::QPStaticStability;
          using constraints::QPStaticStabilityPtr_t;
          typedef constraints::QPStaticStability::ForceData ForceData;
          using model::CenterOfMassComputation;
          using model::CenterOfMassComputationPtr_t;

          std::vector <ForceData> fds;

          for (CORBA::ULong i = 0; i < shapesName.length(); ++i) {
            JointAndShapes_t l = robot->get <JointAndShapes_t>
              (std::string (shapesName[i]));
            if (l.empty ()) throw Error ("Robot shapes not found.");
            for (JointAndShapes_t::const_iterator it = l.begin ();
                it != l.end(); it++) {
              ConvexShape c (ConvexShape (it->second, it->first));
              ForceData fd;
              fd.joint = c.joint_;
              fd.supportJoint = NULL;
              fd.normal = - c.N_;
              fd.points = c.Pts_;
              fds.push_back (fd);
            }
          }

          CenterOfMassComputationPtr_t com = CenterOfMassComputation::create
            (robot);
          com->add (robot->rootJoint ());
          com->computeMass ();
          QPStaticStabilityPtr_t c = QPStaticStability::create (placName, robot,
              fds, com);
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
            DevicePtr_t robot = getRobotOrThrow (problemSolver_);
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
	  (*q_ptr) [(ULong) i] = (*config) [i];
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
        graph::EdgePtr_t edge;
        try {
          edge = HPP_DYNAMIC_PTR_CAST(graph::Edge,
                  graph::GraphComponent::get((size_t)IDedge).lock ());
          if (!edge) {
            std::stringstream ss;
            ss << "ID " << IDedge << " is not an edge";
            std::string errmsg = ss.str();
            throw Error (errmsg.c_str());
          }
        } catch (std::exception& e ) {
          throw Error (e.what());
        }

	bool success = false;
	ConfigurationPtr_t config = floatSeqToConfig (problemSolver_, input);
        ConfigurationPtr_t qoffset = floatSeqToConfig (problemSolver_, qnear);
        try {
	  success = edge->applyConstraints (*qoffset, *config);

          ConstraintSetPtr_t constraint =
            problemSolver_->constraintGraph ()->configConstraint (edge);
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
	  (*q_ptr) [(ULong) i] = (*config) [i];
	}
	output = q_ptr;
	return success;
      }

      bool Problem::buildAndProjectPath (hpp::ID IDedge,
          const hpp::floatSeq& qb,
          const hpp::floatSeq& qe,
          CORBA::Long& indexNotProj,
          CORBA::Long& indexProj)
        throw (hpp::Error)
      {
        /// First get the constraint.
        graph::EdgePtr_t edge;
        try {
          edge = HPP_DYNAMIC_PTR_CAST(graph::Edge,
                  graph::GraphComponent::get((size_t)IDedge).lock ());
          if (!edge) {
            std::stringstream ss;
            ss << "ID " << IDedge << " is not an edge";
            std::string errmsg = ss.str();
            throw Error (errmsg.c_str());
          }
        } catch (std::exception& e ) {
          throw Error (e.what());
        }

	bool success = false;
	ConfigurationPtr_t q1 = floatSeqToConfig (problemSolver_, qb);
        ConfigurationPtr_t q2 = floatSeqToConfig (problemSolver_, qe);
        core::PathVectorPtr_t pv;
        indexNotProj = -1;
        indexProj = -1;
        try {
          core::PathPtr_t path;
	  success = edge->build (path, *q1, *q2, *problemSolver_->problem()->steeringMethod ()->distance());
          if (!success) return false;
          pv = HPP_DYNAMIC_PTR_CAST (core::PathVector, path);
          indexNotProj = problemSolver_->paths ().size ();
          if (!pv) {
            pv = core::PathVector::create (path->outputSize (),
                path->outputDerivativeSize ());
            pv->appendPath (path);
          }
          problemSolver_->addPath (pv);

          core::PathPtr_t projPath;
          success = problemSolver_->problem()->pathProjector ()->apply (path, projPath);

          if (!success) {
            if (!projPath || projPath->length () == 0)
              return false;
          }
          pv = HPP_DYNAMIC_PTR_CAST (core::PathVector, projPath);
          indexProj = problemSolver_->paths ().size ();
          if (!pv) {
            pv = core::PathVector::create (projPath->outputSize (),
                projPath->outputDerivativeSize ());
            pv->appendPath (projPath);
          }
          problemSolver_->addPath (pv);
	} catch (const std::exception& exc) {
	  throw hpp::Error (exc.what ());
	}
	return success;
      }
    } // namespace impl
  } // namespace manipulation
} // namespace hpp
