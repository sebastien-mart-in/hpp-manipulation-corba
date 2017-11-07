#!/usr/bin/env python
#
# Copyright (c) 2017 CNRS
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

import re
from robot import CorbaClient
from constraints import Constraints

def zip_idx (a): return zip(range(len(a)), a)

def _removeEmptyConstraints (problem, constraints):
    return [ n for n in constraints if problem.getConstraintDimensions(n)[2] > 0 ]

class Rules(object):
    def __init__ (self, grippers, handles, rules):
        rs = []
        status = []
        for r in rules:
            handlesRegex = [ None ] * len(grippers)
            for j, gr in zip(range(len(r.grippers)), r.grippers):
                grc = re.compile (gr)
                for i, g in zip(range(len(grippers)), grippers):
                    if grc.match (g):
                        assert handlesRegex[i] is None
                        handlesRegex[i] = re.compile(r.handles[j])
            status.append(r.link)

            rs.append (tuple(handlesRegex))
        self.rules = tuple(rs)
        self.status = tuple(status)
        self.handles = tuple(handles)
        self.defaultAcceptation = False

    def __call__ (self, grasps):
        for r, s in zip(self.rules, self.status):
            apply = True
            for i, h in zip_idx(r):
                if h is not None and not h.match("" if grasps[i] is None else self.handles[grasps[i]]):
                    # This rule does not apply
                    apply = False
                    break
            if apply: return s
        return self.defaultAcceptation

## This class is used to build constraint graph from semantic information.
#
# The minimal usage is the following:
# \code
# graph = ConstraintGraph (robot, "graph")
#
# # Required calls
# factory = ConstraintGraphFactory (graph)
# factory.setGrippers (["gripper1", ... ])
# factory.setObjects (["object1", ], [ [ "object1/handle1", ... ] ], [ [] ])
#
# # Optionally
# factory.environmentContacts (["contact1", ... ])
# factory.setRules ([ Rule (["gripper1", ..], ["handle1", ...], True), ... ])
#
# factory.generate ()
# # graph is initialized
# \endcode
#
# The behaviour can be tuned by setting the functions:
# - ConstraintGraphFactory.graspIsAllowed (redundant with ConstraintGraphFactory.setRules)
# - ConstraintGraphFactory.buildGraspConstraints
# - ConstraintGraphFactory.buildPlacementConstraints
class ConstraintGraphFactory(object):
    class StateAndManifold:
        def __init__ (self, factory, grasps, id, name):
            self.grasps = grasps
            self.id = id
            self.name = name
            self.manifold = Constraints()
            self.foliation = Constraints()
            # Add the grasps
            for ig, ih in zip_idx(grasps):
                if ih is not None:
                    self.manifold += factory.graspConstraints(ig, ih)
                    self.foliation += factory.graspComplementConstraints(ig, ih)
            # Add the placement constraints
            for io, object in zip_idx(factory.objects):
                if not factory._isObjectGrasped(grasps, io):
                    self.manifold += factory.placementConstraints(object)
                    self.foliation += factory.placementComplementConstraints(object)

    ## \param graph an instance of ConstraintGraph
    def __init__(self, graph):
        ## \name Behaviour tuning
        # \{

        ## Reduces the problem combinatorial.
        # Function called to check whether a grasps is allowed.
        # It takes as input a list of handle indices (or None) such
        # that i-th ConstraintGraphFactory.grippers grasps grasps[i]-th ConstraintGraphFactory.handles.
        # It must return a boolean
        #
        # It defaults to a lambda function returning True
        self.graspIsAllowed = lambda x : True
        ## Function called to create grasp constraints.
        # Must return a tuple of Constraints objects as:
        # - constraint that validates the grasp
        # - constraint that parameterizes the graph
        # - constraint that validates the pre-grasp
        # It defaults to ConstraintGraphFactory.defaultBuildGraspConstraints
        self.buildGraspConstraints = self.defaultBuildGraspConstraints
        self._graspConstraints = dict()
        ## Function called to create placement constraints.
        # Must return a tuple of Constraints objects as:
        # - constraint that validates the placement
        # - constraint that parameterizes the  placement
        # - constraint that validates the pre-placement
        # It defaults to ConstraintGraphFactory.defaultBuildPlacementConstraints
        self.buildPlacementConstraints = self.defaultBuildPlacementConstraints
        self._placementConstraints = dict()

        ## \}

        ## \name Internal variables
        # \{

        self.states = dict()
        self.transitions = set()
        ## The ConstraintGraph object to build
        self.graph = graph
        ## the handle names
        self.handles = tuple() # strings
        ## the gripper names
        self.grippers = tuple() # strings
        ## the names of contact on the environment
        self.envContacts = tuple () # strings
        ## the object names
        self.objects = tuple () # strings
        ## See ConstraintGraphFactory.setObjects
        self.handlesPerObjects = tuple () # object index to handle indixes
        ## See ConstraintGraphFactory.setObjects
        self.objectFromHandle = tuple ()  # handle index to object index
        ## See ConstraintGraphFactory.setObjects
        self.contactsPerObjects = tuple ()# object index to contact names

        ## \}

    ## \name Main API
    # \{

    ## \param grippers list of gripper names to be considered
    def setGrippers(self, grippers):
        assert isinstance (grippers, (list, tuple))
        self.grippers = tuple(grippers)

    ## \param objects list of object names to be considered
    ## \param handlesPerObjects a list of list of handle names.
    ## \param contactsPerObjects a list of list of contact names.
    ##  handlesPerObjects and contactsPerObjects must have one list for each object, in the same order.
    def setObjects(self, objects, handlesPerObjects, contactsPerObjects):
        self.objects = tuple(objects)
        handles = []
        hpo = []
        cpo = []
        ofh = []
        for io, o in zip_idx (self.objects):
            hpo.append( tuple(range(len(handles), len(handles) + len(handlesPerObjects[io])) ) )
            handles.extend(handlesPerObjects[io])
            ofh.extend( [ io, ] * len(handlesPerObjects[io]) )
            cpo.append( tuple(contactsPerObjects[io]) )

        self.handles = tuple(handles)
        self.handlesPerObjects = tuple(hpo)
        self.objectFromHandle = tuple(ofh)
        self.contactsPerObjects = tuple(cpo)

    ## \param envContacts contact on the environment to be considered.
    def environmentContacts (self, envContacts):
        self.envContacts = tuple(envContacts)

    ## Set the function ConstraintGraphFactory.graspIsAllowed
    ## \param rules a list of Rule objects
    def setRules (self, rules):
        self.graspIsAllowed = Rules(self.grippers, self.handles, rules)

    ## Go through the combinatorial defined by the grippers and handles
    # and create the states and transitions.
    def generate(self):
        grasps = ( None, ) * len(self.grippers)
        self._recurse(self.grippers, self.handles, grasps, 0)

    ## \}

    ## \name Accessors to the different elementary constraints
    # \{
    def _getGraspConstraints(self, gripper, handle):
        if isinstance(gripper, str): ig = self.grippers.index(gripper)
        else: ig = gripper
        if isinstance(handle, str): ih = self.handles.index(handle)
        else: ih = handle
        k = (ig, ih)
        if not self._graspConstraints.has_key(k):
            self._graspConstraints[k] = self.buildGraspConstraints(self.grippers[ig], self.handles[ih])
        return self._graspConstraints[k]

    def graspConstraints(self, gripper, handle):
        return self._getGraspConstraints(gripper, handle)[0]

    def graspComplementConstraints(self, gripper, handle):
        return self._getGraspConstraints(gripper, handle)[1]

    def pregraspConstraints(self, gripper, handle):
        return self._getGraspConstraints(gripper, handle)[2]

    def _getPlacementConstraints(self, object):
        if isinstance(object, str): io = self.objects.index(object)
        else: io = object
        k = io
        if not self._placementConstraints.has_key(k):
            self._placementConstraints[k] = self.buildPlacementConstraints(self.objects[io])
        return self._placementConstraints[k]

    def placementConstraints(self, object):
        return self._getPlacementConstraints(object)[0]

    def placementComplementConstraints(self, object):
        return self._getPlacementConstraints(object)[1]

    def prePlacementConstraints(self, object):
        return self._getPlacementConstraints(object)[2]
    ## \}

    ## \name Default functions
    # \{

    ## Calls ConstraintGraph.createGraph and ConstraintGraph.createPreGrasp
    ## \param g gripper string
    ## \param h handle  string
    def defaultBuildGraspConstraints (self, g, h):
        n = g + " grasps " + h
        pn = g + " pregrasps " + h
        self.graph.createGrasp (n, g, h)
        self.graph.createPreGrasp (pn, g, h)
        return (Constraints (numConstraints = _removeEmptyConstraints(self.graph.clientBasic.problem, [ n, ])),
                Constraints (numConstraints = _removeEmptyConstraints(self.graph.clientBasic.problem, [ n + "/complement", ])),
                Constraints (numConstraints = _removeEmptyConstraints(self.graph.clientBasic.problem, [ pn, ])),)

    ## This implements strict placement manifolds,
    ## where the parameterization constraints is the complement
    ## of the placement constraint.
    ## \param o string
    def defaultBuildPlacementConstraints (self, o):
        n = "place_" + o
        pn = "preplace_" + o
        width = 0.05
        io = self.objects.index(o)
        if len(self.contactsPerObjects[io]) == 0 or len(self.envContacts) == 0:
            ljs = []
            for n in self.graph.clientBasic.robot.getJointNames():
                if n.startswith(o + "/"):
                    ljs.append(n)
                    q = self.graph.clientBasic.robot.getJointConfig(n)
                    self.graph.clientBasic.problem.createLockedJoint(n, n, q)
            return (Constraints (), Constraints (lockedJoints = ljs), Constraints (),)
        self.graph.client.problem.createPlacementConstraint (n, self.contactsPerObjects[io], self.envContacts)
        self.graph.client.problem.createPrePlacementConstraint (pn, self.contactsPerObjects[io], self.envContacts, width)
        return (Constraints (numConstraints = _removeEmptyConstraints(self.graph.clientBasic.problem, [ n, ])),
                Constraints (numConstraints = _removeEmptyConstraints(self.graph.clientBasic.problem, [ n + "/complement", ])),
                Constraints (numConstraints = _removeEmptyConstraints(self.graph.clientBasic.problem, [ pn, ])),)

    ## This implements relaxed placement manifolds,
    ## where the parameterization constraints is the LockedJoint of
    ## the object root joint
    ## \param o string
    def defaultBuildRelaxedPlacementConstraints (self, o):
        n = "place_" + o
        pn = "preplace_" + o
        width = 0.05
        io = self.objects.index(o)
        ljs = []
        for n in self.graph.clientBasic.robot.getJointNames():
            if n.startswith(o + "/"):
                ljs.append(n)
                q = self.graph.clientBasic.robot.getJointConfig(n)
                self.graph.clientBasic.problem.createLockedJoint(n, n, q)
        if len(self.contactsPerObjects[io]) == 0 or len(self.envContacts) == 0:
            return (Constraints (), Constraints (lockedJoints = ljs), Constraints (),)
        self.graph.client.problem.createPlacementConstraint (n, self.contactsPerObjects[io], self.envContacts)
        self.graph.client.problem.createPrePlacementConstraint (pn, self.contactsPerObjects[io], self.envContacts, width)
        return (Constraints (numConstraints = _removeEmptyConstraints(self.graph.clientBasic.problem, [ n, ])),
                Constraints (lockedJoints = ljs),
                Constraints (numConstraints = _removeEmptyConstraints(self.graph.clientBasic.problem, [ pn, ])),)

    def defaultGraspIsAllowed (self, grasps):
        return True

    ## \}

    ## \name Algorithm main steps
    # \{

    def makeState(self, grasps, priority):
        if not self.states.has_key(grasps):
            # Create state
            name = self._stateName (grasps)
            nid = self.graph.createNode (name, False, priority)
            state = ConstraintGraphFactory.StateAndManifold (self, grasps, nid, name)
            self.states[grasps] = state

            # Create loop transition
            self.makeLoopTransition (state)

            # Add the constraints
            self.graph.addConstraints (node = name, constraints = state.manifold)
        return self.states[grasps]

    def makeLoopTransition(self, state):
        n = self._loopTransitionName (state.grasps)
        self.graph.createEdge (state.name, state.name, n, weight = 0, isInNode = state.name)
        self.graph.addConstraints (edge = n, constraints = state.foliation)

    def makeTransition(self, grasps, nGrasps, ig, priority):
        sf = self.makeState(grasps , priority)
        st = self.makeState(nGrasps, priority)
        names = self._transitionNames(sf, st, ig)
        if names in self.transitions:
            return

        iobj = self.objectFromHandle [nGrasps[ig]]
        obj = self.objects[iobj]
        noPlace = self._isObjectGrasped (grasps, iobj)

        gc = self.graspConstraints (ig, nGrasps[ig])
        gcc = self.graspComplementConstraints (ig, nGrasps[ig])
        pgc = self.pregraspConstraints (ig, nGrasps[ig])
        if noPlace:
            pc = Constraints()
            pcc = Constraints()
            ppc = Constraints()
        else:
            pc = self.placementConstraints (self.objectFromHandle[nGrasps[ig]])
            pcc = self.placementComplementConstraints (self.objectFromHandle[nGrasps[ig]])
            ppc = self.prePlacementConstraints (self.objectFromHandle[nGrasps[ig]])
        manifold = sf.manifold - pc

        # The different cases:
        pregrasp = not pgc.empty()
        intersec = (not gc.empty()) and (not pc.empty())
        preplace = not ppc.empty()

        nWaypoints = pregrasp + intersec + preplace
        nTransitions = 1 + nWaypoints
        nStates = 2 + nWaypoints

        def _createWaypointState (name, constraints):
            self.graph.createNode (name, True)
            self.graph.addConstraints (node = name, constraints = constraints)
            return name

        # Create waypoint states
        intersection = 0
        wStates = [ sf.name, ]
        if pregrasp:
            wStates.append (_createWaypointState (names[0] + "_pregrasp",
                pc  + pgc + manifold))
        if intersec:
            wStates.append (_createWaypointState (names[0] + "_intersec",
                pc  +  gc + manifold))
        if preplace:
            wStates.append (_createWaypointState (names[0] + "_preplace",
                ppc +  gc + manifold))
        wStates.append(st.name)

        # Link waypoints
        transitions = names[:]
        if nWaypoints > 0:
            self.graph.createWaypointEdge (sf.name, st.name, names[0], nWaypoints, automaticBuilder = False)
            self.graph.createWaypointEdge (st.name, sf.name, names[1], nWaypoints, automaticBuilder = False)
            wTransitions = []
            for i in range(nTransitions):
                nf = "{0}_{1}{2}".format(names[0], i, i+1)
                nb = "{0}_{2}{1}".format(names[1], i, i+1)
                self.graph.createEdge (wStates[i], wStates[i+1], nf, -1)
                self.graph.createEdge (wStates[i+1], wStates[i], nb, -1)
                self.graph.graph.setWaypoint (self.graph.edges[transitions[0]],
                        i, self.graph.edges[nf], self.graph.nodes[wStates[i+1]])
                self.graph.graph.setWaypoint (self.graph.edges[transitions[1]],
                        nTransitions - 1 - i, self.graph.edges[nb], self.graph.nodes[wStates[i]])
                wTransitions.append ( (nf, nb) )

            # Set states
            M = 0 if gc.empty() else 1 + pregrasp
            for i in range(M):
                self.graph.setContainingNode (wTransitions[i][0], sf.name)
                self.graph.addConstraints (edge = wTransitions[i][0], constraints = sf.foliation)
                self.graph.setContainingNode (wTransitions[i][1], sf.name)
                self.graph.addConstraints (edge = wTransitions[i][1], constraints = sf.foliation)
            for i in range(M, nTransitions):
                self.graph.setContainingNode (wTransitions[i][0], st.name)
                self.graph.addConstraints (edge = wTransitions[i][0], constraints = st.foliation)
                self.graph.setContainingNode (wTransitions[i][1], st.name)
                self.graph.addConstraints (edge = wTransitions[i][1], constraints = st.foliation)

            # Set all to short except first one.
            for i in range(nTransitions - 1):
                self.graph.setShort (wTransitions[i + 1][0], True)
                self.graph.setShort (wTransitions[i    ][1], True)
        else:
            #TODO This case will likely never happen
            raise NotImplementedError("This case has not been implemented")
            self.graph.createEdge (sf.name, st.name, names[0])
            self.graph.createEdge (st.name, sf.name, names[1])

        self.transitions.add(names)

    ## \}

    def _isObjectGrasped(self, grasps, object):
        for h in self.handlesPerObjects[object]:
            if h in grasps:
                return True
        return False

    def _stateName (self, grasps, abbrev = False):
        sepGH = "-" if abbrev else " grasps "
        sep = ":" if abbrev else " : "
        name = sep.join([ (str(ig) if abbrev else self.grippers[ig]) + sepGH + (str(ih) if abbrev else self.handles[ih]) for ig,ih in zip_idx(grasps) if ih is not None ])
        if len(name) == 0: return "f" if abbrev else "free"
        return name

    def _transitionNames (self, sFrom, sTo, ig):
        g = self.grippers[ig]
        h = self.handles[sTo.grasps[ig]]
        sep = " | "
        return (g + " > " + h + sep + self._stateName(sFrom.grasps, True),
                g + " < " + h + sep + self._stateName(sTo.grasps, True),)

    def _loopTransitionName (self, grasps):
        return "Loop | " + self._stateName(grasps, True)

    def _recurse(self, grippers, handles, grasps, depth):
        if len(grippers) == 0 or len(handles) == 0: return

        isAllowed = self.graspIsAllowed (grasps)
        if isAllowed: self.makeState (grasps, depth)

        for ig, g in zip_idx(grippers):
            ngrippers = grippers[:ig] + grippers[ig+1:]

            isg = self.grippers.index(g)
            for ih, h in zip_idx(handles):
                nhandles = handles[:ih] + handles[ih+1:]

                ish = self.handles.index(h)
                nGrasps = grasps[:isg] + (ish, ) + grasps[isg+1:]

                nextIsAllowed = self.graspIsAllowed (nGrasps)
                if nextIsAllowed: self.makeState (nGrasps, depth + 1)

                if isAllowed and nextIsAllowed:
                    self.makeTransition (grasps, nGrasps, isg, depth)

                self._recurse (ngrippers, nhandles, nGrasps, depth + 2)
