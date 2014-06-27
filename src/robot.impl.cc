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

#include <fcl/math/transform.h>
#include <hpp/util/debug.hh>
#include <hpp/model/urdf/util.hh>
#include <hpp/manipulation/object.hh>
#include <hpp/manipulation/handle.hh>
#include <hpp/model/gripper.hh>
#include <hpp/manipulation/axial-handle.hh>
#include "robot.impl.hh"

namespace hpp {
  namespace manipulation {
    namespace impl {
      Robot::Robot () : problemSolver_ (0x0)
      {
      }

      void Robot::loadRobotModel (const char* robotName,
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

      void Robot::loadHumanoidModel (const char* robotName,
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

      void Robot::loadObjectModel (const char* objectName,
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
	  hppDout (info, *object);
	} catch (const std::exception& exc) {
	  throw Error (exc.what ());
	}
      }

      void Robot::buildCompositeRobot (const char* robotName,
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
      void Robot::addHandle (const char* objectName, const char* linkName,
			     const char* handleName,
			     const ::hpp::Transform localPosition)
	throw (hpp::Error)
      {
	try {
	  ObjectPtr_t object = problemSolver_->object (objectName);
	  JointPtr_t joint = object->getJointByBodyName (linkName);
	  fcl::Quaternion3f q (localPosition [3], localPosition [4],
			       localPosition [5], localPosition [6]);
	  fcl::Vec3f v (localPosition [0], localPosition [1],
			 localPosition [2]);
	  HandlePtr_t handle = Handle::create (handleName, Transform3f (q, v),
					       joint);
	  object->addHandle (handle);
	  /// If manipulation::robot is build, add handle to it
	  if ( problemSolver_->robot() ) 
          { 
    	    HandlePtr_t handleRobot = handle->clone ();
	    handleRobot->name (object->name () + "/" + handle->name ());
	    handleRobot->joint (problemSolver_->robot()->joint(handle
                                                                ->joint ()));
	    problemSolver_->robot()->addHandle (handleRobot->name (),
                                                  handleRobot);
          }
	  hppDout (info, *object);
	} catch (const std::exception& exc) {
	  throw Error (exc.what ());
	}
      }

      void Robot::addGripper(const char* robotName, const char* linkName,
			     const char* gripperName,
			     const ::hpp::Transform handlePositioninJoint,
                             const Names_t& bodyInCollisionNames)
	throw (hpp::Error)
      {
	try {
	  DevicePtr_t robot = problemSolver_->robot (robotName);
	  JointPtr_t joint = robot->getJointByBodyName(linkName);
	  fcl::Quaternion3f q (handlePositioninJoint [3],
                               handlePositioninJoint [4],
			       handlePositioninJoint [5],
                               handlePositioninJoint [6]);
	  fcl::Vec3f v (handlePositioninJoint [0], handlePositioninJoint [1],
			 handlePositioninJoint [2]);
          model::JointVector_t jointInCollision;
          for (CORBA::ULong i=0; i<bodyInCollisionNames.length (); ++i) {     
	    std::string bodyName (bodyInCollisionNames [i]);
            jointInCollision.push_back(robot->getJointByBodyName(bodyName));
          }
	  GripperPtr_t gripper = model::Gripper::create (gripperName, joint, 
                                                  Transform3f (q, v),
                                                  jointInCollision);
	  robot->addGripper (gripper);
          hppDout (info, "add Gripper to robot " << robotName 
                          << " : "<< gripper); 
          /// If manipulation::robot is build, add gripper to it
          if ( problemSolver_->robot() ) 
          { 
    	    GripperPtr_t gripperRobot = gripper->clone ();
	    gripperRobot->name (robot->name () + "/" + gripper->name ());
	    gripperRobot->joint (problemSolver_->robot()
                                  ->joint(gripper->joint ()));
            gripperRobot->removeAllDisabledCollisions();
            model::JointVector_t joints = gripper->getDisabledCollisions();
            for (model::JointVector_t::iterator itJoint = joints.begin() ;
                  itJoint != joints.end() ; itJoint++ ) {
              gripperRobot->addDisabledCollision(problemSolver_->
                                                   robot()->joint(*itJoint));
            }  
	    problemSolver_->robot()->addGripper (gripperRobot->name (),
                                                   gripperRobot);
            deleteGripperCollisions(gripperRobot);
          }
	  hppDout (info, *robot);
	} catch (const std::exception& exc) {
	  throw Error (exc.what ());
	}
      }

      void Robot::deleteGripperCollisions(GripperPtr_t& gripper)
      {
        model::JointVector_t joints = gripper->getDisabledCollisions();
        Objects_t objects = problemSolver_->robot()->objects();
        JointPtr_t joint1;
        JointPtr_t joint2;
        Object::Handles_t handles;
        for (Objects_t::iterator itObject = objects.begin() ;
               itObject != objects.end() ; itObject++) {
          handles = (*itObject)->handles();
          for (model::JointVector_t::iterator itJoint = joints.begin() ;
                 itJoint != joints.end() ; itJoint++ ) {
            joint1 = (*itJoint);
            for (Object::Handles_t::iterator itHandle = handles.begin() ;
                   itHandle != handles.end() ; itHandle++) {
              joint2 = problemSolver_->robot()->joint((*itHandle)->joint());
              problemSolver_->robot()->removeCollisionPairs(joint1, joint2,
                                           hpp::model::COLLISION);
              problemSolver_->robot()->removeCollisionPairs(joint1, joint2,
                                           hpp::model::DISTANCE);
            }
          }
        }
      }

      void Robot::addAxialHandle (const char* objectName, const char* linkName,
				  const char* handleName,
				  const ::hpp::Transform localPosition)
	throw (hpp::Error)
      {
	try {
	  ObjectPtr_t object = problemSolver_->object (objectName);
	  JointPtr_t joint = object->getJointByBodyName (linkName);
	  fcl::Quaternion3f q (localPosition [3], localPosition [4],
			       localPosition [5], localPosition [6]);
	  fcl::Vec3f v (localPosition [0], localPosition [1],
			 localPosition [2]);
	  HandlePtr_t handle = AxialHandle::create
	    (handleName, Transform3f (q, v), joint);
	  object->addHandle (handle);
	  hppDout (info, *object);
	} catch (const std::exception& exc) {
	  throw Error (exc.what ());
	}
      }
      
      Names_t* Robot::getDeviceNames () 
        throw (hpp::Error)
      {
        try {
          std::vector<std::string> nameVector = problemSolver_
                                                 ->robot()->getDeviceNames();
          size_type size = nameVector.size();
          char** nameList = Names_t::allocbuf(size);
	  Names_t *robotNames = new Names_t (size, size, nameList);
          for (size_type it=0; it < size ; it++) {
            nameList [it] =
		(char*) malloc (sizeof(char)*(nameVector[it].length ()+1));
	    strcpy (nameList [it], nameVector[it].c_str ());
          }
          return robotNames;
        } catch (const std::exception& exc) {
	  throw hpp::Error (exc.what ());
	}
      }

      Names_t* Robot::getDeviceJointNames (const char* inDeviceName)
        throw (hpp::Error)
      {
	try {
          std::string deviceName(inDeviceName);
	  DevicePtr_t robot = problemSolver_->robot (deviceName);
	  // Compute number of real urdf joints
	  size_type size = 0;
	  hpp::model::JointVector_t jointVector = robot->getJointVector ();
	  for (hpp::model::JointVector_t::const_iterator it =
                                                      jointVector.begin ();
	       it != jointVector.end (); it++) {
	    if ((*it)->numberDof () != 0) size ++;
	  }
	  char** nameList = Names_t::allocbuf(size);
	  Names_t *jointNames = new Names_t (size, size, nameList);
	  std::size_t rankInConfig = 0;
	  for (std::size_t i = 0; i < jointVector.size (); ++i) {
	    const JointPtr_t joint = jointVector [i];
	    std::string name = joint->name ();
	    std::size_t dimension = joint->numberDof ();
	    if (dimension != 0) {
	      nameList [rankInConfig] =
		(char*) malloc (sizeof(char)*(name.length ()+1));
	      strcpy (nameList [rankInConfig], name.c_str ());
	      ++rankInConfig;
	    }
	  }
	  return jointNames;    
        } catch (const std::exception& exc) {
	  throw hpp::Error (exc.what ());
	}
      }

      char* Robot::getRootBody(const char* inRootJointType, 
                               const char* inDeviceName)
        throw (hpp::Error)
      {
        std::string deviceName(inDeviceName);
        std::string rootJointType(inRootJointType);
        JointPtr_t rootJoint;
        DevicePtr_t device = problemSolver_->robot(deviceName);
        if ( rootJointType == "freeflyer" )
          rootJoint = device->getJointByName("base_joint_SO3");
        if ( rootJointType == "planar" )
          rootJoint = device->getJointByName("base_joint_rz");
        if ( rootJointType == "anchor" )
          rootJoint = device->rootJoint();
        std::string rootBodyName = rootJoint->linkedBody()->name();
        char* name = (char*) malloc (sizeof(char)*(rootBodyName.length()+1));
        strcpy (name, rootBodyName.c_str());
        return name;
      }
    } // namespace impl
  } // namespace manipulation
} // namespace hpp
