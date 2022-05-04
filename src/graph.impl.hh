// Copyright (c) 2014, LAAS-CNRS
// Authors: Joseph Mirabel (joseph.mirabel@laas.fr)
//

// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are
// met:
//
// 1. Redistributions of source code must retain the above copyright
//    notice, this list of conditions and the following disclaimer.
//
// 2. Redistributions in binary form must reproduce the above copyright
// notice, this list of conditions and the following disclaimer in the
// documentation and/or other materials provided with the distribution.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
// "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
// LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
// A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
// HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
// SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
// LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
// DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
// THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
// (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
// OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH
// DAMAGE.

#ifndef HPP_MANIPULATION_CORBA_GRAPH_IMPL_HH
#define HPP_MANIPULATION_CORBA_GRAPH_IMPL_HH

#include <hpp/manipulation/graph/graph.hh>
#include <hpp/manipulation/problem-solver.hh>

#include "hpp/corbaserver/manipulation/fwd.hh"
#include "hpp/corbaserver/manipulation/graph-idl.hh"

namespace hpp {
namespace manipulation {
namespace impl {
using CORBA::Long;
using hpp::corbaserver::manipulation::Namess_t;
using hpp::corbaserver::manipulation::Rules;

class Graph : public virtual POA_hpp::corbaserver::manipulation::Graph {
 public:
  Graph();
  void setServer(Server* server) { server_ = server; }

  virtual Long createGraph(const char* graphName);

  virtual void deleteGraph(const char* graphName);

  virtual void selectGraph(const char* graphName);

  virtual void createSubGraph(const char* subgraphName);

  virtual void setTargetNodeList(const ID graphId, const hpp::IDseq& nodes);

  virtual Long createNode(const Long subGraphId, const char* nodeName,
                          const bool waypoint, const Long priority);

  virtual Long createEdge(const Long nodeFromId, const Long nodeToId,
                          const char* edgeName, const Long weight,
                          const Long isInNodeId);

  virtual Long createWaypointEdge(const Long nodeFromId, const Long nodeToId,
                                  const char* edgeBaseName, const Long number,
                                  const Long weight, const Long isInNodeId);

  virtual void setWaypoint(const ID waypointEdgeId, const Long index,
                           const ID edgeId, const ID nodeId);

  virtual void getGraph(GraphComp_out graph, GraphElements_out elmts);

  virtual void getEdgeStat(ID edgeId, Names_t_out reasons, intSeq_out freqs);

  virtual Long getFrequencyOfNodeInRoadmap(
      ID nodeId, intSeq_out freqPerConnectedComponent);

  virtual bool getConfigProjectorStats(ID elmt, ConfigProjStat_out config,
                                       ConfigProjStat_out path);

  virtual Long getWaypoint(const Long edgeId, const Long index,
                           hpp::ID_out nodeId);

  virtual Long createLevelSetEdge(const Long nodeFromId, const Long nodeToId,
                                  const char* edgeName, const Long w,
                                  const Long isInNodeId);

  virtual void setContainingNode(const ID edgeId, const ID nodeId);

  virtual char* getContainingNode(const ID edgeId);

  virtual void addLevelSetFoliation(const Long edgeId,
                                    const hpp::Names_t& condNC,
                                    const hpp::Names_t& paramNC);

  virtual void setNumericalConstraints(const Long graphComponentId,
                                       const hpp::Names_t& constraintNames) {
    addNumericalConstraints(graphComponentId, constraintNames);
  }
  virtual void addNumericalConstraints(const Long graphComponentId,
                                       const hpp::Names_t& constraintNames);

  virtual void getNumericalConstraints(const Long elmtId,
                                       hpp::Names_t_out names);

  virtual void resetConstraints(const Long graphComponentId);

  virtual void setNumericalConstraintsForPath(
      const Long nodeId, const hpp::Names_t& constraintNames) {
    addNumericalConstraintsForPath(nodeId, constraintNames);
  }
  virtual void addNumericalConstraintsForPath(
      const Long nodeId, const hpp::Names_t& constraintNames);

  virtual void removeCollisionPairFromEdge(ID edgeId, const char* joint1,
                                           const char* joint2);

  virtual void getNode(const hpp::floatSeq& dofArray, ID_out output);

  virtual bool applyNodeConstraints(hpp::ID id, const hpp::floatSeq& input,
                                    hpp::floatSeq_out output,
                                    double& residualError);

  virtual bool applyEdgeLeafConstraints(hpp::ID IDedge,
                                        const hpp::floatSeq& qleaf,
                                        const hpp::floatSeq& input,
                                        hpp::floatSeq_out output,
                                        double& residualError);

  virtual bool generateTargetConfig(hpp::ID IDedge, const hpp::floatSeq& qleaf,
                                    const hpp::floatSeq& input,
                                    hpp::floatSeq_out output,
                                    double& residualError);

  virtual CORBA::Boolean getConfigErrorForNode(ID nodeId,
                                               const hpp::floatSeq& dofArray,
                                               hpp::floatSeq_out error);

  virtual CORBA::Boolean getConfigErrorForEdge(ID edgeId,
                                               const hpp::floatSeq& dofArray,
                                               hpp::floatSeq_out error);

  virtual CORBA::Boolean getConfigErrorForEdgeLeaf(
      ID edgeId, const hpp::floatSeq& leafDofArray,
      const hpp::floatSeq& dofArray, hpp::floatSeq_out error);

  virtual CORBA::Boolean getConfigErrorForEdgeTarget(
      ID edgeId, const hpp::floatSeq& leafDofArray,
      const hpp::floatSeq& dofArray, hpp::floatSeq_out error);

  virtual void displayNodeConstraints(hpp::ID nodeId,
                                      CORBA::String_out constraints);

  virtual void displayEdgeConstraints(hpp::ID edgeId,
                                      CORBA::String_out constraints);

  virtual void displayEdgeTargetConstraints(hpp::ID edgeId,
                                            CORBA::String_out constraints);

  virtual void getNodesConnectedByEdge(hpp::ID edgeId, CORBA::String_out from,
                                       CORBA::String_out to);

  virtual void display(const char* filename);

  virtual void getHistogramValue(ID edgeId, hpp::floatSeq_out freq,
                                 hpp::floatSeqSeq_out values);

  virtual void setShort(ID edgeId, CORBA::Boolean isShort);

  virtual bool isShort(ID edgeId);

  virtual Long autoBuild(const char* graphName, const Names_t& grippers,
                         const Names_t& objects,
                         const Namess_t& handlesPerObject,
                         const Namess_t& shapesPreObject,
                         const Names_t& envNames, const Rules& rulesList);

  virtual void setWeight(ID edgeId, const Long weight);

  virtual Long getWeight(ID edgeId);

  virtual char* getName(ID elmtId);

  virtual void initialize();

  virtual void getRelativeMotionMatrix(ID edgeID, intSeqSeq_out matrix);
  virtual void setSecurityMarginForEdge(ID edgeId, const char* joint1,
                                        const char* joint2, double margin);
  virtual void getSecurityMarginMatrixForEdge(ID edgeId,
                                              floatSeqSeq_out matrix);

 private:
  template <typename T>
  shared_ptr<T> getComp(ID id, bool throwIfWrongType = true);
  ProblemSolverPtr_t problemSolver();
  graph::GraphPtr_t graph(bool throwIfNull = true);
  Server* server_;
};  // class Graph
}  // namespace impl
}  // namespace manipulation
}  // namespace hpp

#endif  // HPP_MANIPULATION_CORBA_GRAPH_IMPL_HH
