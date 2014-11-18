#!/usr/bin/env python
#
# Copyright (c) 2014 CNRS
# Author: Joseph Mirabel
#
# This file is part of hpp-manipulation-corba.
# hpp-manipulation-corba is free software: you can redistribute it
# and/or modify it under the terms of the GNU Lesser General Public
# License as published by the Free Software Foundation, either version
# 3 of the License, or (at your option) any later version.
#
# hpp-manipulation-corba is distributed in the hope that it will be
# useful, but WITHOUT ANY WARRANTY; without even the implied warranty
# of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
# General Lesser Public License for more details.  You should have
# received a copy of the GNU Lesser General Public License along with
# hpp-manipulation-corba.  If not, see
# <http://www.gnu.org/licenses/>.

### Definition of a constraint graph.
##
##  This class wraps the Corba client to the server implemented by
##  libhpp-manipulation-corba.so
##
##  Some method implemented by the server can be considered as private. The
##  goal of this class is to hide them and to expose those that can be
##  considered as public.

from subprocess import Popen

class ConstraintGraph (object):
    dotCmd = ['dot', '-Tpdf']
    pdfviewCmd = ['evince']

    def __init__ (self, robot, graphName):
        self.client = robot.client.manipulation
        self.clientBasic = robot.client.basic
        self.graph = robot.client.manipulation.graph
        self.name = graphName
        self.grasps = dict ()
        self.pregrasps = dict ()
        self.nodes = dict ()
        self.edges = dict ()
        self.graphId = self.graph.createGraph (graphName)
        self.subGraphId = self.graph.createSubGraph (graphName + "_sg")

    ### Display the current graph.
    ## The graph is printed in DOT format. Command dot must be
    ## available.
    ## \param dotOut full path of the generated DOT file.
    ## \param pdfOut fill path of the generated PDF document.
    ## \note DOT and PDF files will be overwritten and are not automatically
    ## deleted so you can keep them.
    def display (self, dotOut = '/tmp/constraintgraph.dot', pdfOut = '/tmp/constraintgraph.pdf'):
        self.graph.display (dotOut)
        dotCmd = self.dotCmd[:]
        dotCmd.append ('-o' + pdfOut)
        dotCmd.append (dotOut)
        dot = Popen (dotCmd)
        dot.wait ()
        pdfviewCmd = self.pdfviewCmd[:]
        pdfviewCmd.append (pdfOut)
        Popen (pdfviewCmd)

    ### Create one or several node
    ## \param node name (resp. list of names) of the node(s) to be created.
    ## \note The order is important. The first should be the most restrictive one as a configuration
    ## will be in the first node for which the constraint are satisfied.
    def createNode (self, node):
        if type (node) is str:
            node = [node]
        for n in node:
            self.nodes [n] = self.graph.createNode (self.subGraphId, n)

    ### Create an edge
    ## \param nodeFrom, nodeTo the extremities of the edge,
    ## \param name name of the edge,
    ## \param weight see note,
    ## \param isInNodeFrom true if a path corresponding to this edge lies in nodeFrom;
    ##       this will be automatically set if you leave it.
    ## \note The weights define the probability of selecting an edge among all the
    ## outgoing edges of a node. The probability of an edge is \f$ \frac{w_i}{\sum_j{w_j}} \f$,
    ## where each \f$ w_j \f$ corresponds to an outgoing edge from a given node.
    ## To have an edge that cannot be selected by the M-RRT algorithm but is still acceptable,
    ## set its weight to zero.
    def createEdge (self, nodeFrom, nodeTo, name, weight = 1, isInNodeFrom = None):
        if isInNodeFrom is None:
            isInNodeFrom = (self.nodes[nodeFrom] > self.nodes[nodeTo])
        self.edges [name] =\
            self.graph.createEdge (self.nodes[nodeFrom], self.nodes[nodeTo], name, weight, isInNodeFrom)

    ### Create a WaypointEdge.
    ## \param nodeFrom, nodeTo, name, weight, isInNodeFrom see createEdge note,
    ## \param nb number of waypoints,
    ## \return an object containing two list:
    ##         \code
    ##           returnValue.edges
    ##           returnValue.nodes
    ##         \endcode
    ##         Each element of each list contains a name and an id.
    ##         Nevertheless, the information is stored in the instance of this class.
    ## \note See documentation of class hpp::manipulation::graph::WaypointEdge for more information.
    def createWaypointEdge (self, nodeFrom, nodeTo, name, nb = 1, weight = 1, isInNodeFrom = None):
        if isInNodeFrom is None:
            isInNodeFrom = (self.nodes[nodeFrom] > self.nodes[nodeTo])
        elmts = self.graph.createWaypointEdge (self.nodes[nodeFrom], self.nodes[nodeTo], name, nb, weight, isInNodeFrom)
        for e in elmts.edges:
            self.edges [e.name] = e.id
        for n in elmts.nodes:
            self.nodes [n.name] = n.id
        return elmts

    ### Create a LevelSetEdge.
    ## \param nodeFrom, nodeTo, name, weight, isInNodeFrom see createEdge note.
    ## \note See documentation of class hpp::manipulation::graph::LevelSetEdge for more information.
    def createLevelSetEdge (self, nodeFrom, nodeTo, name, weight = 1, isInNodeFrom = None):
        if isInNodeFrom is None:
            isInNodeFrom = (self.nodes[nodeFrom] > self.nodes[nodeTo])
        self.edges [name] =\
            self.graph.createLevelSetEdge (self.nodes[nodeFrom], self.nodes[nodeTo], name, weight, isInNodeFrom)

    ### Create grasp constraints between a gripper and a handle
    ## \param name basename of the constraints,
    ## \param gripper, handle names of the gripper and handle,
    ## \param passiveJoints a list of joints that should not be modified by the constraint.
    ## \note The passive joints are only used for path constraints and are for optimization only.
    def createGrasp (self, name, gripper, handle, passiveJoints = None):
        self.client.problem.createGrasp (name, gripper, handle)
        self.grasps [(name, False)] = [name]
        if passiveJoints is not None:
            self.client.problem.createGrasp (name + "_passive", gripper, handle)
            self.clientBasic.problem.setPassiveDofs (name + "_passive", passiveJoints)
            self.grasps [(name, True)] = [name + "_passive"]
        else:
            self.grasps [(name, True)] = [name]

    ### Create pre-grasp constraints between a gripper and a handle
    ## See createGrasp.
    def createPreGrasp (self, name, gripper, handle, passiveJoints = None):
        self.client.problem.createPreGrasp (name, gripper, handle)
        self.pregrasps [(name, False)] = [name, name + "/0_f_0.05"]
        if passiveJoints is not None:
            self.client.problem.createPreGrasp (name + "_passive", gripper, handle)
            self.clientBasic.problem.setPassiveDofs (name + "_passive", passiveJoints)
            self.pregrasps [(name, True)] = [name + "_passive", name + "_passive/0_f_0.05"]
        else:
            self.pregrasps [(name, True)] = [name, name + "/0_f_0.05"]

    ### Set the constraints of an edge or a node.
    ## This method sets the constraints of an element of the graph and handles the
    ## special cases of grasps constraints.
    ## \param graph set to true if you are defining constraints for every nodes,
    ## \param node, edge name of a component of the graph,
    ## \param grasp, pregrasp name, or list of names, of grasp or pregrasp.
    ## \note Exaclty one of the parameter graph, node and edge must be set.
    def setConstraints (self, graph = False, node = None, edge = None, grasp = None, pregrasp = None, lockDof = [], numConstraints = []):
        nc = numConstraints [:]
        nc_p = numConstraints [:]
        if grasp is not None:
            if type(grasp) is str:
                grasp = [grasp]
            for g in grasp:
                nc.extend (self.grasps[(g, False)])
                nc_p.extend (self.grasps[(g, True)])
        if pregrasp is not None:
            if type(pregrasp) is str:
                pregrasp = [pregrasp]
            for g in pregrasp:
                nc.extend (self.pregrasps[(g, False)])
                nc_p.extend (self.pregrasps[(g, True)])

        if node is not None:
            self.graph.setNumericalConstraints (self.nodes [node], nc)
            self.graph.setNumericalConstraintsForPath (self.nodes [node], nc_p)
            self.graph.setLockedDofConstraints (self.nodes [node], lockDof)
        elif edge is not None:
            self.graph.setNumericalConstraints (self.edges [edge], nc)
            self.graph.setLockedDofConstraints (self.edges [edge], lockDof)
        elif graph:
            self.graph.setNumericalConstraints (self.graphId, nc)
            self.graph.setLockedDofConstraints (self.graphId, lockDof)
