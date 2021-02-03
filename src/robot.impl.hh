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

#ifndef HPP_MANIPULATION_CORBA_ROBOT_IMPL_HH
# define HPP_MANIPULATION_CORBA_ROBOT_IMPL_HH

# include <hpp/corbaserver/manipulation/fwd.hh>
# include <hpp/manipulation/problem-solver.hh>
# include "hpp/corbaserver/manipulation/robot-idl.hh"

namespace hpp {
  namespace manipulation {
    namespace impl {
      using CORBA::Short;

      class Robot : public virtual POA_hpp::corbaserver::manipulation::Robot
      {
        public:
          Robot ();

          void setServer (Server* server)
          {
            server_ = server;
          }

          virtual void insertRobotModel (const char* robotName,
              const char* rootJointType, const char* urdfName,
              const char* srdfName);

          virtual void insertRobotModelOnFrame (const char* robotName,
              const char* frameName, const char* rootJointType,
              const char* urdfName, const char* srdfName);

          virtual void insertRobotModelFromString (const char* robotName,
              const char* rootJointType,
              const char* urdfString,
              const char* srdfString);

          virtual void insertRobotSRDFModel (const char* robotName,
              const char* srdfPath);

          virtual void insertRobotSRDFModelFromString (const char* robotName,
              const char* srdfString);

          virtual void insertHumanoidModel (const char* robotName,
              const char* rootJointType, const char* urdfName,
              const char* srdfName);

          virtual void insertHumanoidModelFromString (const char* robotName,
              const char* rootJointType,
              const char* urdfString,
              const char* srdfString);

          virtual void loadEnvironmentModel (const char* urdfName,
              const char* srdfName, const char* prefix);

          virtual void loadEnvironmentModelFromString (const char* urdfString,
              const char* srdfString, const char* prefix);

          virtual Transform__slice* getRootJointPosition (const char* robotName);

          virtual void setRootJointPosition (const char* robotName,
              const ::hpp::Transform_ position);

          virtual void addHandle (const char* linkName, const char* handleName,
              const ::hpp::Transform_ localPosition);

          virtual void addGripper(const char* linkName, const char* gripperName,
              const ::hpp::Transform_ handlePositioninJoint);

          virtual void addAxialHandle (const char* linkName,
              const char* handleName, const ::hpp::Transform_ localPosition);

          virtual char* getGripperPositionInJoint (const char* gripperName,
              ::hpp::Transform__out position);

          virtual char* getHandlePositionInJoint (const char* handleName,
              ::hpp::Transform__out position);

        private:
          ProblemSolverPtr_t problemSolver();
          Server* server_;
      }; // class Robot
    } // namespace impl
  } // namespace manipulation
} // namespace hpp

#endif // HPP_MANIPULATION_CORBA_ROBOT_IMPL_HH
