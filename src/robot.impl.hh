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
# include "robot.hh"

namespace hpp {
  namespace manipulation {
    namespace impl {
      using CORBA::Short;

      class Robot : public virtual POA_hpp::corbaserver::manipulation::Robot
      {
      public:
	Robot ();
	void setProblemSolver (const ProblemSolverPtr_t& problemSolver)
	{
	  problemSolver_ = problemSolver;
	}

	virtual void loadRobotModel (const char* robotName,
				     const char* rootJointType,
				     const char* packageName,
				     const char* modelName,
				     const char* urdfSuffix,
				     const char* srdfSuffix)
	  throw (hpp::Error);

	virtual void loadHumanoidModel (const char* robotName,
					const char* rootJointType,
					const char* packageName,
					const char* modelName,
					const char* urdfSuffix,
					const char* srdfSuffix)
	  throw (hpp::Error);

	virtual void loadObjectModel (const char* objectName,
				      const char* rootJointType,
				      const char* packageName,
				      const char* modelName,
				      const char* urdfSuffix,
				      const char* srdfSuffix)
	  throw (hpp::Error);

	virtual void buildCompositeRobot (const char* robotName,
					  const hpp::Names_t& robotNames)
	  throw (hpp::Error);


	virtual void addHandle (const char* objectName, const char* linkName,
				const char* handleName,
				const ::hpp::Transform localPosition)
	  throw (hpp::Error);
        virtual void addGripper(const char* robotName,
                                const char* linkName,
			        const char* gripperName,
			        const ::hpp::Transform handlePositioninJoint)
          throw (hpp::Error);

	virtual void addAxialHandle (const char* objectName,
				     const char* linkName,
				     const char* handleName,
				     const ::hpp::Transform localPosition)
	  throw (hpp::Error);
private:
	ProblemSolverPtr_t problemSolver_;
      }; // class Robot
    } // namespace impl
  } // namespace manipulation
} // namespace hpp

#endif // HPP_MANIPULATION_CORBA_ROBOT_IMPL_HH
