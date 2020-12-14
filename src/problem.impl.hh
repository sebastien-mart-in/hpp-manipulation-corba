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

#ifndef HPP_MANIPULATION_CORBA_PROBLEM_IMPL_HH
# define HPP_MANIPULATION_CORBA_PROBLEM_IMPL_HH

# include <hpp/corbaserver/manipulation/fwd.hh>
# include <hpp/manipulation/problem-solver.hh>
# include "hpp/corbaserver/manipulation/problem-idl.hh"

# include "hpp/manipulation_idl/_graph-idl.hh"
# include "hpp/manipulation_idl/_path_planners-idl.hh"

namespace hpp {
  namespace manipulation {
    namespace impl {
      using CORBA::Short;
      using CORBA::UShort;
      using CORBA::ULong;
      using CORBA::Double;
      using CORBA::String_out;

      class Problem : public virtual POA_hpp::corbaserver::manipulation::Problem
      {
      public:
	Problem ();
        void setServer (Server* server)
        {
          server_ = server;
        }

        virtual bool selectProblem (const char* name);

        virtual void resetProblem ();

        virtual Names_t* getAvailable (const char* what);

        virtual Names_t* getSelected (const char* what);

        virtual void loadRoadmap (const char* filename);

        virtual void createGrasp (const char* graspName,
            const char* gripperName, const char* handleName);

        virtual void createPreGrasp (const char* graspName,
            const char* gripperName, const char* handleName);

        virtual Names_t* getEnvironmentContactNames ();

        virtual Names_t* getRobotContactNames ();

        virtual Names_t* getEnvironmentContact (const char* name,
            intSeq_out indexes, floatSeqSeq_out points);

        virtual Names_t* getRobotContact (const char* name,
            intSeq_out indexes, floatSeqSeq_out points);

        virtual void createPlacementConstraint (const char* placName,
            const Names_t& shapeName, const Names_t& envContactName);

        virtual void createPrePlacementConstraint (const char* placName,
            const Names_t& shapeName, const Names_t& envContactName,
            CORBA::Double width);

        virtual void createQPStabilityConstraint (const char* placName,
            const Names_t& shapesName);

        virtual bool setConstraints (hpp::ID id, bool target);

        virtual void registerConstraints
        (const char* constraint, const char* complement, const char* both);

        virtual bool applyConstraints (hpp::ID id, const hpp::floatSeq& input,
            hpp::floatSeq_out output, double& residualError);

        virtual bool applyConstraintsWithOffset (hpp::ID IDedge,
            const hpp::floatSeq& qnear, const hpp::floatSeq& input,
            hpp::floatSeq_out output, double& residualError);

        virtual bool buildAndProjectPath (hpp::ID IDedge,
            const hpp::floatSeq& qb,
            const hpp::floatSeq& qe,
            CORBA::Long& indexNotProj,
            CORBA::Long& indexProj);

        virtual void setTargetState (hpp::ID IDstate);

        virtual ID edgeAtParam (ULong pathId, Double param, String_out name);

        hpp::manipulation_idl::graph_idl::Validation_ptr createGraphValidation ();

        core_idl::Roadmap_ptr readRoadmap(const char* filename,
            pinocchio_idl::Device_ptr robot, manipulation_idl::graph_idl::Graph_ptr graph);
        core_idl::Roadmap_ptr writeRoadmap(const char* filename,
            pinocchio_idl::Device_ptr robot, manipulation_idl::graph_idl::Graph_ptr graph);

        core_idl::Roadmap_ptr createRoadmap(core_idl::Distance_ptr distance, pinocchio_idl::Device_ptr robot);
      private:
        ProblemSolverPtr_t problemSolver();
        graph::GraphPtr_t graph(bool throwIfNull = true);
        Server* server_;
      }; // class Problem
    } // namespace impl
  } // namespace manipulation
} // namespace hpp

#endif // HPP_MANIPULATION_CORBA_PROBLEM_IMPL_HH
