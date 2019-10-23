// Copyright (c) 2014, LAAS-CNRS
// Authors: Joseph Mirabel (joseph.mirabel@laas.fr)
//
// This file is part of hpp-manipulation.
// hpp-manipulation is free software: you can redistribute it
// and/or modify it under the terms of the GNU Lesser General Public
// License as published by the Free Software Foundation, either version
// 3 of the License, or (at your option) any later version.
//
// hpp-manipulation is distributed in the hope that it will be
// useful, but WITHOUT ANY WARRANTY; without even the implied warranty
// of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
// General Lesser Public License for more details.  You should have
// received a copy of the GNU Lesser General Public License along with
// hpp-manipulation. If not, see <http://www.gnu.org/licenses/>.

#ifndef HPP_MANIPULATION_CORBA_GRAPH_IMPL_HH
# define HPP_MANIPULATION_CORBA_GRAPH_IMPL_HH

# include <hpp/manipulation/problem-solver.hh>
# include <hpp/manipulation/graph/graph.hh>

# include "hpp/corbaserver/manipulation/fwd.hh"
# include "hpp/corbaserver/manipulation/graph-idl.hh"

namespace hpp {
  namespace manipulation {
    namespace impl {
      using hpp::corbaserver::manipulation::Namess_t;
      using hpp::corbaserver::manipulation::Rules;
      using CORBA::Long;

      class Graph : public virtual POA_hpp::corbaserver::manipulation::Graph
      {
        public:
          Graph ();
          void setServer (Server* server)
          {
            server_ = server;
          }

          virtual Long createGraph(const char* graphName)
            throw (hpp::Error);

          virtual void deleteGraph(const char* graphName)
            throw (hpp::Error);

          virtual void selectGraph(const char* graphName)
            throw (hpp::Error);

          virtual void createSubGraph(const char* subgraphName)
            throw (hpp::Error);

          virtual void setTargetNodeList(const ID graphId, const hpp::IDseq& nodes)
            throw (hpp::Error);

          virtual Long createNode (const Long subGraphId,
                                   const char* nodeName,
                                   const bool waypoint,
                                   const Long priority)
            throw (hpp::Error);

          virtual Long createEdge (const Long nodeFromId,
                                   const Long nodeToId,
                                   const char* edgeName,
                                   const Long weight,
                                   const Long isInNodeId)
            throw (hpp::Error);

          virtual Long createWaypointEdge (const Long nodeFromId,
                                           const Long nodeToId,
                                           const char* edgeBaseName,
                                           const Long number,
                                           const Long weight,
                                           const Long isInNodeId)
            throw (hpp::Error);

          virtual void setWaypoint (const ID waypointEdgeId, const Long index,
              const ID edgeId, const ID nodeId)
            throw (hpp::Error);

          virtual void getGraph (GraphComp_out graph, GraphElements_out elmts)
            throw (hpp::Error);

          virtual void getEdgeStat (ID edgeId,
              Names_t_out reasons, intSeq_out freqs)
            throw (hpp::Error);

          virtual Long getFrequencyOfNodeInRoadmap (ID nodeId, intSeq_out freqPerConnectedComponent)
            throw (hpp::Error);

          virtual bool getConfigProjectorStats (ID elmt, ConfigProjStat_out config,
              ConfigProjStat_out path)
            throw (hpp::Error);

          virtual Long getWaypoint (const Long edgeId, const Long index,
              hpp::ID_out nodeId)
            throw (hpp::Error);

          virtual Long createLevelSetEdge(const Long nodeFromId,
                                          const Long nodeToId,
                                          const char* edgeName,
                                          const Long w,
                                          const Long isInNodeId)
            throw (hpp::Error);

          virtual void setContainingNode (const ID edgeId, const ID nodeId)
            throw (hpp::Error);

          virtual char* getContainingNode (const ID edgeId)
            throw (hpp::Error);

          virtual void addLevelSetFoliation (const Long edgeId,
                                             const hpp::Names_t& condNC,
                                             const hpp::Names_t& paramNC)
            throw (hpp::Error);

          virtual void setNumericalConstraints (const Long graphComponentId,
                                       const hpp::Names_t& constraintNames,
                                       const hpp::Names_t& passiveDofsNames)
            throw (hpp::Error)
          {
            addNumericalConstraints(graphComponentId,
                constraintNames, passiveDofsNames);
          }
          virtual void addNumericalConstraints (const Long graphComponentId,
                                       const hpp::Names_t& constraintNames,
                                       const hpp::Names_t& passiveDofsNames)
            throw (hpp::Error);

	  virtual void getNumericalConstraints(const Long elmtId, hpp::Names_t_out names)
	    throw(hpp::Error);

	  virtual void resetConstraints(const Long graphComponentId) 
	    throw (hpp::Error);

          virtual void setNumericalConstraintsForPath (const Long nodeId,
              const hpp::Names_t& constraintNames,
              const hpp::Names_t& passiveDofsNames)
            throw (hpp::Error)
          {
            addNumericalConstraintsForPath(nodeId,
                constraintNames, passiveDofsNames);
          }
          virtual void addNumericalConstraintsForPath (const Long nodeId,
              const hpp::Names_t& constraintNames,
              const hpp::Names_t& passiveDofsNames)
            throw (hpp::Error);

          virtual void removeCollisionPairFromEdge
	  (ID edgeId, const char* joint1, const char* joint2)
	    throw (hpp::Error);

          virtual void getNode (const hpp::floatSeq& dofArray, ID_out output)
            throw (hpp::Error);

        virtual bool applyNodeConstraints
        (hpp::ID id, const hpp::floatSeq& input,
         hpp::floatSeq_out output, double& residualError)
          throw (hpp::Error);

        virtual bool applyEdgeLeafConstraints
        (hpp::ID IDedge, const hpp::floatSeq& qleaf, const hpp::floatSeq& input,
         hpp::floatSeq_out output, double& residualError)
          throw (hpp::Error);

        virtual bool generateTargetConfig
        (hpp::ID IDedge, const hpp::floatSeq& qleaf, const hpp::floatSeq& input,
         hpp::floatSeq_out output, double& residualError)
          throw (hpp::Error);

	virtual CORBA::Boolean getConfigErrorForNode
	(ID nodeId, const hpp::floatSeq& dofArray, hpp::floatSeq_out error)
	  throw (hpp::Error);

	virtual CORBA::Boolean getConfigErrorForEdge
	(ID edgeId, const hpp::floatSeq& dofArray, hpp::floatSeq_out error)
	  throw (hpp::Error);

	virtual CORBA::Boolean getConfigErrorForEdgeLeaf
	(ID edgeId, const hpp::floatSeq& leafDofArray,
	 const hpp::floatSeq& dofArray, hpp::floatSeq_out error)
	  throw (hpp::Error);

	virtual CORBA::Boolean getConfigErrorForEdgeTarget
	(ID edgeId, const hpp::floatSeq& leafDofArray,
	 const hpp::floatSeq& dofArray, hpp::floatSeq_out error)
	  throw (hpp::Error);

	virtual void displayNodeConstraints
	(hpp::ID nodeId, CORBA::String_out constraints) throw (Error);

	virtual void displayEdgeConstraints
	(hpp::ID edgeId, CORBA::String_out constraints) throw (Error);

	virtual void displayEdgeTargetConstraints
	(hpp::ID edgeId, CORBA::String_out constraints) throw (Error);

	virtual void getNodesConnectedByEdge
	(hpp::ID edgeId, CORBA::String_out from, CORBA::String_out to)
	  throw (Error);

          virtual void display (const char* filename)
            throw (hpp::Error);

          virtual void getHistogramValue (ID edgeId, hpp::floatSeq_out freq,
              hpp::floatSeqSeq_out values)
            throw (hpp::Error);

          virtual void setShort (ID edgeId, CORBA::Boolean isShort)
            throw (hpp::Error);

          virtual bool isShort (ID edgeId)
            throw (hpp::Error);

          virtual Long autoBuild (const char* graphName,
              const Names_t& grippers, const Names_t& objects,
              const Namess_t& handlesPerObject, const Namess_t& shapesPreObject,
	      const Names_t& envNames, const Rules& rulesList)
            throw (hpp::Error);

          virtual void setWeight (ID edgeId, const Long weight)
            throw (hpp::Error);

          virtual Long getWeight (ID edgeId)
            throw (hpp::Error);

          virtual char* getName (ID elmtId)
            throw (hpp::Error);

          virtual void initialize ()
            throw (hpp::Error);

          virtual void getRelativeMotionMatrix (ID edgeID, intSeqSeq_out matrix)
            throw (hpp::Error);

        private:
          template <typename T> boost::shared_ptr<T> getComp(ID id, bool throwIfWrongType = true);
          ProblemSolverPtr_t problemSolver();
          graph::GraphPtr_t graph(bool throwIfNull = true);
          Server* server_;
      }; // class Graph
    } // namespace impl
  } // namespace manipulation
} // namespace hpp

#endif // HPP_MANIPULATION_CORBA_GRAPH_IMPL_HH
