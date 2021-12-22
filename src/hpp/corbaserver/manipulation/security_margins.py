#!/usr/bin/env python
#
# Copyright (c) 2020 CNRS, Airbus SAS
# Author: Florent Lamiraux
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

## Handles security margins between robots and objects in collision checking
#
#  This class sets the requested security margins in \link
#  hpp::core::PathValidation PathValidation\endlink instances of
#  constraints graph \link hpp::manipulation::graph::Edges
#  edges\endlink.
#
#  \sa hpp::core::ObstacleUserInterface::setSecurityMargins
#
#  By default the security margin between objects or robots is set to 0
#  It can be modified by changing the value of the class member
#  \c defaultMargin.
#
#  A given security margin can be set for a pair of robots or objects by calling
#  method \link SecurityMargins.setSecurityMarginBetween
#  \c setSecurityMarginBetween \endlink.
#
#  Once appropriate margins have been set by the user, method
#  \link SecurityMargins.apply \c apply \endlink computes and sets for each edge
#  and each pair of bodies security margins as required. However, along edges
#  where two objects are requested to be close to each other, the security
#  margin is set to 0. For instance
#  \li between a pregrasp and grasp configuration the security margin is set to
#      0 between the gripper and the object to be grasped,
#  \li along an edge where an object lies in a stable position the security
#      margin between this object and any object holding a surface over which
#      it could be put is set to 0.
class SecurityMargins(object):
    defaultMargin = 0
    ## Constructor
    #  \param robotsAndObjects list of robots and objects. It is assumed that
    #         joints names are prefixed by these name followed by "/"
    def __init__(self, problemSolver, factory, robotsAndObjects):
        self.ps = problemSolver
        self.robot = self.ps.robot
        self.factory = factory
        self.robotsAndObjects = robotsAndObjects
        self.marginMatrix = dict()
        self.computeJoints()
        self.computeGrippers()
        self.computePossibleContacts()

    def computeJoints(self):
        self.robotToJoints = dict()
        for ro in self.robotsAndObjects:
            l = len(ro)
            self.robotToJoints[ro] = list(filter(lambda n:n[:l+1] == ro + "/",
                                            self.robot.jointNames))
        self.robotToJoints["universe"] = ['universe']
        self.jointToRobot = dict()
        for ro, joints in self.robotToJoints.items():
            for j in joints:
                self.jointToRobot[j] = ro

    def computeGrippers(self):
        grippers = self.ps.getAvailable("gripper")
        self.gripperToRobot = dict()
        self.gripperToJoints = dict()
        for g in grippers:
            j = self.ps.robot.getGripperPositionInJoint(g)[0]
            self.gripperToRobot[g] = self.jointToRobot[j]
            self.gripperToJoints[g] = [j] + self.ps.robot.getChildJoints(j)

    def computePossibleContacts(self):
        names = ["universe"]
        names += self.robot.jointNames
        # separate joint names
        self.contactSurfaces = dict()
        for k in self.robotToJoints.keys():
            self.contactSurfaces[k] = list()
        # Sort contact surfaces by object
        surfaces = self.ps.getEnvironmentContactNames()
        for s in surfaces:
            contacts = self.ps.getEnvironmentContact(s)
            joint = contacts[0][0]
            self.contactSurfaces[self.jointToRobot[joint]].append(s)
        surfaces = self.ps.getRobotContactNames()
        for s in surfaces:
            contacts = self.ps.getRobotContact(s)
            joint = contacts[0][0]
            self.contactSurfaces[self.jointToRobot[joint]].append(s)
        # Compute pair of objects that can be in contact
        self.possibleContacts = list()
        for o1, l1 in self.contactSurfaces.items():
            for o2, l2 in self.contactSurfaces.items():
                if o1 != o2 and len(l1) > 0 and len(l2) > 0:
                    self.possibleContacts.append ((o1,o2))

    ## Set security margin between two robots or objects
    #  \param obj1 name of first robot or object,
    #  \param obj2 name of second robot or object.
    #  \param margin margin to set between those robots or objects.
    def setSecurityMarginBetween(self, obj1, obj2, margin):
        self.marginMatrix[frozenset([obj1, obj2])] = margin

    ## Get security margin between two robots or objects
    #  \param obj1 name of first robot or object,
    #  \param obj2 name of second robot or object.
    def getSecurityMarginBetween(self, obj1, obj2):
        key = frozenset([obj1, obj2])
        return self.marginMatrix.get(key, self.defaultMargin)

    # Get list of constraints that are active somewhere along the edge
    #
    # The result is a dictionary with
    #  - key "place" and value a list of objects,
    #  - key "grasp" and value a list of pairs (gripper, object).
    def getActiveConstraintsAlongEdge (self, edge):
        factory = self.factory; graph = factory.graph
        p = graph.clientBasic.problem.getProblem()
        g = p.getConstraintGraph()
        e = g.get(graph.edges[edge])
        s1 = e.stateFrom()
        s2 = e.stateTo()
        c1 = list(map(lambda c:c.function().name(), s1.numericalConstraints()))
        c1 += list(map(lambda c:c.function().name(), g.numericalConstraints()))
        c2 = list(map(lambda c:c.function().name(), s2.numericalConstraints()))
        c2 += list(map(lambda c:c.function().name(), g.numericalConstraints()))
        d = set(c1).union(set(c2))
        res = dict()
        res["place"] = list()
        res["grasp"] = list()
        for c in d:
            for o in factory.objects:
                if c == "place_" + o:
                    res["place"].append(o)
            for g in factory.grippers:
                for o, handles in zip(factory.objects,
                                      factory.handlesPerObjects):
                    # o object
                    # handles <- indices of handles of object o
                    for h in handles:
                        handle = factory.handles[h]
                        if c == g + " grasps " + handle:
                            res["grasp"].append((g,o))
        return res

    ## Set security margins between
    #  \li robot bodies and objects,
    #  \li robot bodies and environment,
    #  \li objects and environment,
    #
    #  For each edge, do the following:
    #  \li set requested security margin between each pair of robots or
    #      objects,
    #  \li detect grasp and placement constraints that are active sometime along
    #      the edge,
    #  \li for each active grasp constraint, set security margin to 0 between
    #      joints of the gripper and object,
    #  \li for each active placement constraint, set security margin to 0
    #      between the placed object and any object or robot that holds a
    #      contact surface.
    #  \todo take into account environment.
    def apply (self):
        factory = self.factory; graph = factory.graph
        # Set security margin for each edge
        for e in graph.edges.keys():
            # first set requested security margin between each pair of objects
            for i1, ro1 in enumerate(self.robotsAndObjects):
                for i2, ro2 in enumerate(self.robotsAndObjects):
                    if i2 < i1: continue
                    margin = self.getSecurityMarginBetween(ro1, ro2)
                    for k1, j1 in enumerate(self.robotToJoints[ro1]):
                        for k2, j2 in enumerate(self.robotToJoints[ro2]):
                            if j1!=j2:
                                graph.setSecurityMarginForEdge\
                                    (e, j1, j2, margin)
            # Then set 0 margin where necessary.
            # for grasps, set 0 between gripper and object.
            constraints = self.getActiveConstraintsAlongEdge(e)
            for g, ro1 in constraints["grasp"]:
                for j1 in self.robotToJoints[ro1]:
                    for j2 in self.gripperToJoints[g]:
                        graph.setSecurityMarginForEdge(e, j1, j2, 0)
            # For placement set 0 between object and any other object that can
            # be in contact.
            for o1 in constraints["place"]:
                for o2, o3 in self.possibleContacts:
                    if o1 == o2:
                        for j1 in self.robotToJoints[o1]:
                            for j2 in self.robotToJoints[o3]:
                                graph.setSecurityMarginForEdge(e, j1, j2, 0)
