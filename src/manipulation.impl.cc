// Copyright (c) 2012 CNRS
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
#include <hpp/model/urdf/util.hh>
#include <hpp/manipulation/object.hh>
#include "manipulation.impl.hh"

namespace hpp {
  namespace manipulation {
    namespace impl {
      Manipulation::Manipulation () : problemSolver_ (0x0)
      {
      }

      void Manipulation::loadRobotModel (const char* robotName,
					 const char* rootJointType,
					 const char* packageName,
					 const char* modelName,
					 const char* urdfSuffix,
					 const char* srdfSuffix)
	throw (Error)
      {
	try {
	  model::DevicePtr_t robot = model::Device::create
	    (std::string (robotName));
	  model::urdf::loadRobotModel (robot,
				       std::string (rootJointType),
				       std::string (packageName),
				       std::string (modelName),
				       std::string (urdfSuffix),
				       std::string (srdfSuffix));
	  // Add device to the planner
	  problemSolver_->addRobot (robotName, robot);
	} catch (const std::exception& exc) {
	  throw Error (exc.what ());
	}
      }

      void Manipulation::loadHumanoidModel (const char* robotName,
					    const char* rootJointType,
					    const char* packageName,
					    const char* modelName,
					    const char* urdfSuffix,
					    const char* srdfSuffix)
	throw (Error)
      {
	try {
	  model::HumanoidRobotPtr_t robot =
	    model::HumanoidRobot::create (std::string (robotName));
	  model::urdf::loadHumanoidModel (robot,
					  std::string (rootJointType),
					  std::string (packageName),
					  std::string (modelName),
					  std::string (urdfSuffix),
					  std::string (srdfSuffix));
	  // Add device to the planner
	  problemSolver_->addRobot (robotName, robot);
	} catch (const std::exception& exc) {
	  throw Error (exc.what ());
	}
      }

      void Manipulation::loadObjectModel (const char* objectName,
					  const char* rootJointType,
					  const char* packageName,
					  const char* modelName,
					  const char* urdfSuffix,
					  const char* srdfSuffix)
	throw (Error)
      {
	try {
	  manipulation::ObjectPtr_t object = manipulation::Object::create
	    (objectName);
	  model::urdf::loadRobotModel (object,
				       std::string (rootJointType),
				       std::string (packageName),
				       std::string (modelName),
				       std::string (urdfSuffix),
				       std::string (srdfSuffix));
	  // Add device to the planner
	  problemSolver_->addObject (objectName, object);
	} catch (const std::exception& exc) {
	  throw Error (exc.what ());
	}
      }

      void Manipulation::buildCompositeRobot (const char* robotName,
					      const Names_t& robotNames)
	throw (Error)
      {
	try {
	  ProblemSolver::Names_t names;
	  for (CORBA::ULong i=0; i<robotNames.length (); ++i) {
	    names.push_back (std::string (robotNames [i]));
	  }
	  problemSolver_->buildCompositeRobot (std::string (robotName), names);
	} catch (const std::exception& exc) {
	  throw Error (exc.what ());
	}
      }
    } // namespace impl
  } // namespace manipulation
} // namespace hpp
