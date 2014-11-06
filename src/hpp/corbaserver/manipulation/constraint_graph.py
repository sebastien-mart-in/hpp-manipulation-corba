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

    def createEdge (self, nodeFrom, nodeTo, name, weight = 1, isInNodeFrom = None):
        if isInNodeFrom is None:
            isInNodeFrom = (self.nodes[nodeFrom] > self.nodes[nodeTo])
        self.edges [name] =\
            self.graph.createEdge (self.nodes[nodeFrom], self.nodes[nodeTo], name, weight, isInNodeFrom)

    def createWaypointEdge (self, nodeFrom, nodeTo, name, nb = 1, weight = 1, isInNodeFrom = None):
        if isInNodeFrom is None:
            isInNodeFrom = (self.nodes[nodeFrom] > self.nodes[nodeTo])
        elmts = self.graph.createWaypointEdge (self.nodes[nodeFrom], self.nodes[nodeTo], name, nb, weight, isInNodeFrom)
        for e in elmts.edges:
            self.edges [e.name] = e.id
        for n in elmts.nodes:
            self.nodes [n.name] = n.id
        return elmts

    def createLevelSetEdge (self, nodeFrom, nodeTo, name, weight = 1, isInNodeFrom = None):
        if isInNodeFrom is None:
            isInNodeFrom = (self.nodes[nodeFrom] > self.nodes[nodeTo])
        self.edges [name] =\
            self.graph.createLevelSetEdge (self.nodes[nodeFrom], self.nodes[nodeTo], name, weight, isInNodeFrom)

    def createGrasp (self, name, gripper, handle, passiveJoints = None):
        self.client.problem.createGrasp (name, gripper, handle)
        self.grasps [(name, False)] = [name]
        if passiveJoints is not None:
            self.client.problem.createGrasp (name + "_passive", gripper, handle)
            self.clientBasic.problem.setPassiveDofs (name + "_passive", passiveJoints)
            self.grasps [(name, True)] = [name + "_passive"]
        else:
            self.grasps [(name, True)] = [name]

    def createPreGrasp (self, name, gripper, handle, passiveJoints = None):
        self.client.problem.createPreGrasp (name, gripper, handle)
        self.pregrasps [(name, False)] = [name, name + "/ineq_0", name + "/ineq_0.1"]
        if passiveJoints is not None:
            self.client.problem.createPreGrasp (name + "_passive", gripper, handle)
            self.clientBasic.problem.setPassiveDofs (name + "_passive", passiveJoints)
            #self.pregrasps [(name, True)] = [name + "_passive", name + "_passive/ineq_0", name + "_passive/ineq_0.1"]
            self.pregrasps [(name, True)] = [name]
        else:
            #self.pregrasps [(name, True)] = [name, name + "/ineq_0", name + "/ineq_0.1"]
            self.pregrasps [(name, True)] = [name]

    def setConstraints (self, graph = False, node = None, edge = None, grasp = None, pregrasp = None, lockDof = [], numConstraints = []):
        nc = numConstraints [:]
        nc_p = numConstraints [:]
        if grasp is not None:
            nc.extend (self.grasps[(grasp, False)])
            nc_p.extend (self.grasps[(grasp, True)])
        if pregrasp is not None:
            nc.extend (self.pregrasps[(pregrasp, False)])
            nc_p.extend (self.pregrasps[(pregrasp, True)])

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
