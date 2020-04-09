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

import re, abc, sys
from .constraints import Constraints

def _removeEmptyConstraints (problem, constraints):
    return [ n for n in constraints if problem.getConstraintDimensions(n)[2] > 0 ]

class Rules(object):
    def __init__ (self, grippers, handles, rules):
        rs = []
        status = []
        for r in rules:
            # replace empty strings by the corresponding regexp "^$", otherwise "" matches with all strings.
            for i in range(len(r.grippers)):
                if r.grippers[i] == "": r.grippers[i] = "^$"
            for i in range(len(r.handles)):
                if r.handles[i] == "": r.handles[i] = "^$"
            handlesRegex = [ None ] * len(grippers)
            for j, gr in enumerate (r.grippers):
                grc = re.compile (gr)
                for i, g in enumerate (grippers):
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
            for i, h in enumerate(r):
                if h is not None and not h.match("" if grasps[i] is None else self.handles[grasps[i]]):
                    # This rule does not apply
                    apply = False
                    break
            if apply: return s
        return self.defaultAcceptation

if sys.version_info.major == 2:
    class ABC:
        """ Python 2.7 equivalent to abc.ABC Python 3 class."""
        __metaclass__ = abc.ABCMeta
else:
    from abc import ABC

## An abstract class which is loops over the different (gripper, handle) associations.
#
# The behaviour can be tuned by setting the callback functions:
# - \ref graspIsAllowed (redundant with \ref setRules)
# - \ref constraint_graph_factory_algo_callbacks "Algorithm steps"
#
# <b>Sketch of the algorithm</b>
#
# Let
#  \li \f$G\f$ be the set of grippers,
#  \li \f$H\f$ be the set of handles.
#
#  A \b grasp is defined as a pair \f$(g,h)\in G\times H\f$.
#
#  Each \b state (excluding waypoint states) is defined by a set of grasps.
#  Given a set of grasps, we define
#  \li <b>available grippers</b> as the grippers that are not the first
#      element of any pair of the set of grasps,
#  \li <b>available handles</b> as the handles that are not the second element
#     of any pair of the set of grasps.
#
#  The first node is defined by the empty set. The graph is built recursiveley
#  as follows:
#
#  if the set of grasps defining the node is not allowed (method \link
#      constraint_graph_factory.GraphFactoryAbstract.graspIsAllowed
#      graspIsAllowed \endlink), return.
#
#  Otherwise, for any pair \f$(g,h)\f$ of available grippers and available
#  handles,
#  \li build a new state by adding grasp \f$(g,h)\f$ to the current set of
#      grasps (method
#      \link constraint_graph_factory.GraphFactoryAbstract.makeState
#      makeState\endlink),
#  \li build a transition from the current state to the new state (method \link
#      constraint_graph_factory.GraphFactoryAbstract.makeTransition
#      makeTransition \endlink)
#  \li build a loopTransition from the current state to itself (method \link
#      constraint_graph_factory.GraphFactoryAbstract.makeLoopTransition
#      makeLoopTransition \endlink)
#  \li repeat the two above states to the new state.
class GraphFactoryAbstract(ABC):
    def __init__(self):

        ## Reduces the problem combinatorial.
        # Function called to check whether a grasps is allowed.
        # It takes as input a list of handle indices (or None) such
        # that i-th \ref grippers grasps `grasps[i]`-th \ref handles.
        # It must return a boolean
        #
        # It defaults to: \code lambda x : True
        self.graspIsAllowed = lambda x : True

        ## \name Internal variables
        # \{

        self.states = dict()
        self.transitions = set()
        ## the handle names
        self.handles = tuple() # strings
        ## the gripper names
        self.grippers = tuple() # strings
        ## the names of contact on the environment
        self.envContacts = tuple () # strings
        ## the object names
        self.objects = tuple () # strings
        ## See \ref setObjects
        self.handlesPerObjects = tuple () # object index to handle indixes
        ## See \ref setObjects
        self.objectFromHandle = tuple ()  # handle index to object index
        ## See \ref setObjects
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
        if len(objects) != len(handlesPerObjects):
            raise IndexError("There should be as many handlesPerObjects as objects")
        if len(objects) != len(contactsPerObjects):
            raise IndexError("There should be as many contactsPerObjects as objects")
        self.objects = tuple(objects)
        handles = []
        hpo = []
        cpo = []
        ofh = []
        for io, o in enumerate (self.objects):
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

    ## Set the function \ref graspIsAllowed
    ## \param rules a list of Rule objects
    def setRules (self, rules):
        self.graspIsAllowed = Rules(self.grippers, self.handles, rules)

    ## Go through the combinatorial defined by the grippers and handles
    # and create the states and transitions.
    def generate(self):
        grasps = ( None, ) * len(self.grippers)
        self._recurse(self.grippers, self.handles, grasps, 0)

    ## \}

    ## \name Abstract methods of the algorithm
    #  \anchor constraint_graph_factory_algo_callbacks
    # \{

    ## Create a new state.
    # \param grasps a handle index for each gripper, as in GraphFactoryAbstract.graspIsAllowed.
    # \param priority the state priority.
    # \return an object representing the state.
    @abc.abstractmethod
    def makeState(self, grasps, priority): return grasps

    ## Create a loop transition.
    # \param state: an object returned by \ref makeState which represent the state
    @abc.abstractmethod
    def makeLoopTransition(self, state): pass

    ## Create two transitions between two different states.
    # \param stateFrom: same as grasps in \ref makeState
    # \param stateTo: same as grasps in \ref makeState
    # \param ig: index if the grasp vector that changes, i.e. such that
    #   - \f$ stateFrom.grasps[i_g] \neq stateTo.grasps[i_g] \f$
    #   - \f$ \forall i \neq i_g, stateFrom.grasps[i] = stateTo.grasps[i] \f$
    @abc.abstractmethod
    def makeTransition(self, stateFrom, stateTo, ig): pass

    ## \}

    def _makeState(self, grasps, priority):
        if grasps not in self.states:
            state = self.makeState (grasps, priority)
            self.states[grasps] = state

            # Create loop transition
            self.makeLoopTransition (state)
        else:
            state = self.states [grasps]
        return state

    def _isObjectGrasped(self, grasps, object):
        for h in self.handlesPerObjects[object]:
            if h in grasps:
                return True
        return False

    def _stateName (self, grasps, abbrev = False):
        sepGH = "-" if abbrev else " grasps "
        sep = ":" if abbrev else " : "
        name = sep.join([ (str(ig) if abbrev else self.grippers[ig]) + sepGH + (str(ih) if abbrev else self.handles[ih]) for ig,ih in enumerate(grasps) if ih is not None ])
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

    ## Recurse across all possible sets of grasps
    #
    #  This method visits all possible set of grasps and create states
    #  and transitions between those states.
    #
    #  \param grippers list of names of available grippers: that do not hold
    #         an handle yet,
    #  \param handles list of names of available handles that are not hold yet
    #         by a gripper.
    #  \param grasps list of grasps already active. Grasps are represented by
    #         a list of handle indices or None if the gripper is available.
    #         the order in the list corresponds to the order of the gripper
    #         in the list of all grippers.
    #         For instance, if a robot has 3 grippers
    #           ("g1", "g2", "g3"), and grasps is equal to
    #           ("h2", "h1", None), then
    #           \li "g1" holds "h2",
    #           \li "g2" holds "h1", and
    #           \li "g3" does not hold anything.
    #
    def _recurse(self, grippers, handles, grasps, depth):
        isAllowed = self.graspIsAllowed (grasps)
        if isAllowed: current = self._makeState (grasps, depth)

        if len(grippers) == 0 or len(handles) == 0: return
        for ig, g in enumerate(grippers):
            # ngrippers = { grippers } \ grippers[ig]
            ngrippers = grippers[:ig] + grippers[ig+1:]
            # isg <- index of g in list of all grippers
            isg = self.grippers.index(g)
            for ih, h in enumerate(handles):
                # nhandles = { handles } \ handles[ih]
                nhandles = handles[:ih] + handles[ih+1:]
                # ish <- index of h in all handles
                ish = self.handles.index(h)
                # nGrasp <- substitute current handle index at current gripper
                # position.
                nGrasps = grasps[:isg] + (ish, ) + grasps[isg+1:]

                nextIsAllowed = self.graspIsAllowed (nGrasps)
                if nextIsAllowed: next = self._makeState (nGrasps, depth + 1)

                if isAllowed and nextIsAllowed:
                    self.makeTransition (current, next, isg)

                self._recurse (ngrippers, nhandles, nGrasps, depth + 2)

## An abstract class which stores the constraints.
#
# Child classes are responsible for building them.
# - \ref buildGrasp
# - \ref buildPlacement
class ConstraintFactoryAbstract(ABC):
    def __init__(self, graphfactory):
        self._grasp = dict()
        self._placement = dict()

        self.graphfactory = graphfactory

    ## \name Accessors to the different elementary constraints
    # \{

    ## Get constraints relative to a grasp
    #
    #  \param gripper name of a gripper or gripper index
    #  \param handle name of a handle or handle index
    #  \return a dictionary with keys <c>['grasp', 'graspComplement',
    #          'preGrasp']</c> and values the corresponding constraints as
    #          \link manipulation.constraints.Constraints Constraints\endlink
    #          instances.
    #  \warning If grasp does not exist, the function creates it.
    def getGrasp(self, gripper, handle):
        if isinstance(gripper, str): ig = self.graphfactory.grippers.index(gripper)
        else: ig = gripper
        if isinstance(handle, str): ih = self.graphfactory.handles.index(handle)
        else: ih = handle
        k = (ig, ih)
        if k not in self._grasp:
            self._grasp[k] = self.buildGrasp(self.graphfactory.grippers[ig], None if ih is None else self.graphfactory.handles[ih])
            assert isinstance (self._grasp[k], dict)
        return self._grasp[k]

    ## Get constraints relative to a grasp
    #
    #  \param gripper name of a gripper or gripper index,
    #  \param handle name of a handle or handle index,
    #  \param what a word among <c>['grasp', 'graspComplement', 'preGrasp']</c>.
    #  \return the corresponding constraints as
    #          \link manipulation.constraints.Constraints Constraints\endlink
    #          instances.
    #  \warning If grasp does not exist, the function creates it.
    def g (self, gripper, handle, what):
        return self.getGrasp(gripper, handle)[what]

    ## Get constraints relative to an object placement
    #
    #  \param object object name or index
    #  \return a dictionary with keys <c>['placement', 'placementComplement',
    #          'prePlacement']</c> and values the corresponding constraints as
    #          \link manipulation.constraints.Constraints Constraints\endlink
    #          instances.
    #  \warning If placement does not exist, the function creates it.
    def getPlacement(self, object):
        if isinstance(object, str): io = self.graphfactory.objects.index(object)
        else: io = object
        k = io
        if k not in self._placement:
            self._placement[k] = self.buildPlacement(self.graphfactory.objects[io])
        return self._placement[k]

    ## Get constraints relative to a placement
    #
    #  \param object object name or index
    #  \param what a word among <c>['placement', 'placementComplement',
    #         'prePlacement']</c>.
    #  \return the corresponding constraints as
    #          \link manipulation.constraints.Constraints Constraints\endlink
    #          instances.
    #  \warning If placement does not exist, the function creates it.
    def p (self, object, what):
        return self.getPlacement(object)[what]
    ## \}

    ## Function called to create grasp constraints.
    # Must return a tuple of Constraints objects as:
    # - constraint that validates the grasp
    # - constraint that parameterizes the graph
    # - constraint that validates the pre-grasp
    # \param g gripper string
    # \param h handle  string
    @abc.abstractmethod
    def buildGrasp (self, g, h):
        return (None, None, None,)

    ## Function called to create placement constraints.
    # Must return a tuple of Constraints objects as:
    # - constraint that validates placement
    # - constraint that parameterizes placement
    # - constraint that validates pre-placement
    # \param o string
    @abc.abstractmethod
    def buildPlacement (self, o):
        return (None, None, None,)

## Default implementation of ConstraintFactoryAbstract
class ConstraintFactory(ConstraintFactoryAbstract):
    gfields = ('grasp', 'graspComplement', 'preGrasp')
    pfields = ('placement', 'placementComplement', 'prePlacement')

    def __init__ (self, graphfactory, graph):
        super (ConstraintFactory, self).__init__(graphfactory)
        self.graph = graph
        ## Select whether placement should be strict or relaxed.
        # \sa buildStrictPlacement, buildRelaxedPlacement
        self.strict = False

    ## Calls ConstraintGraph.createGraph and ConstraintGraph.createPreGrasp
    ## \param g gripper string
    ## \param h handle  string
    def buildGrasp (self, g, h):
        n = g + " grasps " + h
        pn = g + " pregrasps " + h
        self.graph.createGrasp (n, g, h)
        self.graph.createPreGrasp (pn, g, h)
        return dict ( list(zip (self.gfields, (
            Constraints (numConstraints = _removeEmptyConstraints(self.graph.clientBasic.problem, [ n, ])),
            Constraints (numConstraints = _removeEmptyConstraints(self.graph.clientBasic.problem, [ n + "/complement", ])),
            Constraints (numConstraints = _removeEmptyConstraints(self.graph.clientBasic.problem, [ pn, ])),
            ))))

    def buildPlacement (self, o):
        if self.strict:
            return self.buildStrictPlacement (o)
        else:
            return self.buildRelaxedPlacement (o)

    ## This implements strict placement manifolds,
    ## where the parameterization constraints is the complement
    ## of the placement constraint.
    ## \param o string
    def buildStrictPlacement (self, o):
        n = "place_" + o
        pn = "preplace_" + o
        # Get distance of object to surface in preplacement
        distance = self.graphfactory.getPreplacementDistance(o)
        io = self.graphfactory.objects.index(o)
        placeAlreadyCreated = n in self.graph.clientBasic.problem.getAvailable ("numericalconstraint")
        if (len(self.graphfactory.contactsPerObjects[io]) == 0 or len(self.graphfactory.envContacts) == 0) and not placeAlreadyCreated:
            ljs = []
            for n in self.graph.clientBasic.robot.getJointNames():
                if n.startswith(o + "/"):
                    ljs.append(n)
                    q = self.graph.clientBasic.robot.getJointConfig(n)
                    self.graph.clientBasic.problem.createLockedJoint(n, n, q)
            return dict ( list(zip (self.pfields,
                                    (Constraints (),
                                     Constraints (numConstraints = ljs),
                                     Constraints (),))))
        if not placeAlreadyCreated:
            self.graph.client.problem.createPlacementConstraint (n, self.graphfactory.contactsPerObjects[io], self.graphfactory.envContacts)
        if not pn in self.graph.clientBasic.problem.getAvailable ("numericalconstraint"):
            self.graph.client.problem.createPrePlacementConstraint \
                (pn, self.graphfactory.contactsPerObjects[io],
                 self.graphfactory.envContacts, distance)
        return dict ( list(zip (self.pfields, (
            Constraints (numConstraints = _removeEmptyConstraints(self.graph.clientBasic.problem, [ n, ])),
            Constraints (numConstraints = _removeEmptyConstraints(self.graph.clientBasic.problem, [ n + "/complement", ])),
            Constraints (numConstraints = _removeEmptyConstraints(self.graph.clientBasic.problem, [ pn, ])),
            ))))

    ## This implements relaxed placement manifolds,
    ## where the parameterization constraints is the LockedJoint of
    ## the object root joint
    ## \param o string
    def buildRelaxedPlacement (self, o):
        n = "place_" + o
        pn = "preplace_" + o
        # Get distance of object to surface in preplacement
        distance = self.graphfactory.getPreplacementDistance(o)
        io = self.graphfactory.objects.index(o)
        ljs = []
        for jn in self.graph.clientBasic.robot.getJointNames():
            if jn.startswith(o + "/"):
                ljs.append(jn)
                q = self.graph.clientBasic.robot.getJointConfig(jn)
                self.graph.clientBasic.problem.createLockedJoint(jn, jn, q)
        placeAlreadyCreated = n in self.graph.clientBasic.problem.getAvailable ("numericalconstraint")
        if (len(self.graphfactory.contactsPerObjects[io]) == 0 or len(self.graphfactory.envContacts) == 0) and not placeAlreadyCreated:
            return dict ( list(zip (self.pfields,
                                    (Constraints (),
                                     Constraints (numConstraints = ljs),
                                     Constraints (),))))
        if not placeAlreadyCreated:
            self.graph.client.problem.createPlacementConstraint (n, self.graphfactory.contactsPerObjects[io], self.graphfactory.envContacts)
        if not pn in self.graph.clientBasic.problem.getAvailable ("numericalconstraint"):
            self.graph.client.problem.createPrePlacementConstraint \
                (pn, self.graphfactory.contactsPerObjects[io],
                 self.graphfactory.envContacts, distance)
        return dict ( list(zip (self.pfields,
                                (Constraints (numConstraints = \
                                              _removeEmptyConstraints\
                                              (self.graph.clientBasic.problem,
                                               [ n, ])),
                                 Constraints (numConstraints = ljs),
                                 Constraints (numConstraints = \
                                              _removeEmptyConstraints\
                                              (self.graph.clientBasic.problem,
                                               [ pn, ])),))))

## Default implementation of ConstraintGraphFactory
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
class ConstraintGraphFactory(GraphFactoryAbstract):
    class StateAndManifold:
        def __init__ (self, factory, grasps, id, name):
            self.grasps = grasps
            self.id = id
            self.name = name
            self.manifold = Constraints()
            self.foliation = Constraints()
            # Add the grasps
            for ig, ih in enumerate(grasps):
                if ih is not None:
                    self.manifold += factory.constraints.g (ig, ih, 'grasp')
                    self.foliation += factory.constraints.g (ig, ih, 'graspComplement')
            # Add the placement constraints
            for io, object in enumerate(factory.objects):
                if not factory._isObjectGrasped(grasps, io):
                    self.manifold += factory.constraints.p (object, 'placement')
                    self.foliation += factory.constraints.p (object, 'placementComplement')

    ## defaut distance between objec and surface in preplacement configuration
    defaultPreplaceDist = 0.05
    preplaceDistance = dict()

    ## \param graph an instance of ConstraintGraph
    def __init__(self, graph):
        super (ConstraintGraphFactory, self).__init__()

        ## Stores the constraints in a child class of ConstraintFactoryAbstract
        self.constraints = ConstraintFactory (self, graph)

        self.graph = graph

    ## \name Default functions
    # \{

    def makeState(self, grasps, priority):
        # Create state
        name = self._stateName (grasps)
        nid = self.graph.createNode (name, False, priority)
        state = ConstraintGraphFactory.StateAndManifold (self, grasps, nid, name)

        # Add the constraints
        self.graph.addConstraints (node = name, constraints = state.manifold)
        return state

    def makeLoopTransition(self, state):
        n = self._loopTransitionName (state.grasps)
        self.graph.createEdge (state.name, state.name, n, weight = 0, isInNode = state.name)
        self.graph.addConstraints (edge = n, constraints = state.foliation)

    ## Make transition between to states
    #
    #  \param stateFrom initial state,
    #  \param stateTo new state
    #  \param ig index of gripper
    #
    #  stateTo grasps are the union of stateFrom grasps with a set containing
    #  a grasp by gripper of index ig in list <c>self.grippers</c> of a handle.
    #  The index of the newly grasped handle can be found by
    #  <c>stateTo.grasps[ig]</c>
    def makeTransition(self, stateFrom, stateTo, ig):
        sf = stateFrom
        st = stateTo
        grasps  = sf.grasps
        nGrasps = st.grasps
        names = self._transitionNames(sf, st, ig)
        if names in self.transitions:
            return

        # index of newly grasped handle when crossing the transition
        ih = st.grasps[ig]
        # index of newly grasped object when crossing the transition
        iobj = self.objectFromHandle [ih]
        # whether newly grasped object is already grasped in stateFrom
        noPlace = self._isObjectGrasped (sf.grasps, iobj)

        # Constraints defining the new grasp as a Constraints instance.
        gc  = self.constraints.g (ig, ih, 'grasp')
        # Constraints defining the complement of the new grasp as a Constraints
        # instance.
        gcc = self.constraints.g (ig, ih, 'graspComplement')
        # Constraints defining the new pregrasp as a Constraints instance.
        pgc = self.constraints.g (ig, ih, 'preGrasp')
        if noPlace:
            pc = Constraints()
            pcc = Constraints()
            ppc = Constraints()
        else:
            # If object is placed in stateFrom,
            #   get constraints defining the placement as a Constraints
            #   instance,
            pc  = self.constraints.p (self.objectFromHandle[ih], 'placement')
            #   get constraint defining the complement of the placement as a
            #   Constraints instance,
            pcc = self.constraints.p (self.objectFromHandle[ih], 'placementComplement')
            #   get the preplacement constraint as a Constraints instance.
            ppc = self.constraints.p (self.objectFromHandle[ih], 'prePlacement')
        manifold = sf.manifold - pc

        # The different cases:
        pregrasp = not pgc.empty()
        intersec = (not gc.empty()) and (not pc.empty())
        preplace = not ppc.empty()

        nWaypoints = pregrasp + intersec + preplace
        nTransitions = 1 + nWaypoints
        nStates = 2 + nWaypoints

        # Check whether level set edge is required
        assert (len(sf.foliation.grasps) == 0)
        assert (len(sf.foliation.pregrasps) == 0)
        assert (len(st.foliation.grasps) == 0)
        assert (len(st.foliation.pregrasps) == 0)
        crossedFoliation = False
        if len(sf.foliation.numConstraints) > 0 and \
           len(st.foliation.numConstraints) > 0:
            crossedFoliation = True
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
            if crossedFoliation:
                self.graph.createWaypointEdge\
                    (sf.name, st.name, names[0] + "_ls", nWaypoints, 10,
                     automaticBuilder = False)
                if not noPlace:
                    # If object is already grasped, the backward waypoint edge
                    # with levelset edge is useless
                    self.graph.createWaypointEdge\
                        (st.name, sf.name, names[1] + "_ls", nWaypoints, 10,
                         automaticBuilder = False)
            wTransitions = []
            for i in range(nTransitions):
                nf = "{0}_{1}{2}".format(names[0], i, i+1)
                nb = "{0}_{2}{1}".format(names[1], i, i+1)
                self.graph.createEdge (wStates[i], wStates[i+1], nf, -1)
                self.graph.createEdge (wStates[i+1], wStates[i], nb, -1)
                if crossedFoliation:
                    nf_ls = nf
                    nb_ls = nb
                    # Add LevelSetEdges
                    if i == 0:
                        edgeName = nf_ls = nf + "_ls"
                        # containing state is always start state
                        containingState = sf.name
                        self.graph.createLevelSetEdge(wStates[i], wStates[i+1],
                                                      edgeName, -1,
                                                      containingState)
                        paramNC = (st.foliation - sf.foliation).numConstraints
                        condNC = (st.manifold - sf.manifold).numConstraints
                        self.graph.addLevelSetFoliation\
                            (edgeName, condNC = condNC, paramNC = paramNC)
                        self.graph.addConstraints (edge = edgeName,
                                                   constraints = sf.foliation)
                    if i == nTransitions - 1 and crossedFoliation:
                        edgeName = nb_ls = nb + "_ls"
                        # containing state is goal state if an object in
                        # placement is grasped, start state otherwise.
                        if not noPlace:
                            containingState = st.name
                            self.graph.createLevelSetEdge\
                                (wStates[i+1], wStates[i], edgeName, -1,
                                 containingState)
                            pNC = (sf.foliation - st.foliation).numConstraints
                            cNC = sf.manifold.numConstraints
                            self.graph.addLevelSetFoliation\
                                (edgeName, condNC = cNC, paramNC = pNC)
                            self.graph.addConstraints\
                                (edge = edgeName, constraints = st.foliation)
                    self.graph.graph.setWaypoint\
                        (self.graph.edges[names[0] + '_ls'], i,
                         self.graph.edges[nf_ls],
                         self.graph.nodes[wStates[i+1]])
                    if not noPlace:
                        self.graph.graph.setWaypoint\
                            (self.graph.edges[names[1] + '_ls'],
                             nTransitions-1-i, self.graph.edges[nb_ls],
                             self.graph.nodes[wStates[i]])

                self.graph.graph.setWaypoint (self.graph.edges[names[0]],
                        i, self.graph.edges[nf], self.graph.nodes[wStates[i+1]])
                self.graph.graph.setWaypoint (self.graph.edges[names[1]],
                        nTransitions - 1 - i, self.graph.edges[nb],
                        self.graph.nodes[wStates[i]])
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

            # If pregrasp, add grasp complement constraint to edge linking
            # pregrasp to grasp
            if pregrasp:
                self.graph.addConstraints(edge = wTransitions[1][0],
                                          constraints = gcc)
                self.graph.addConstraints(edge = wTransitions[1][1],
                                          constraints = gcc)
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

    ## \name Tuning the constraints
    #  \{

    ## Set preplacement distance
    #  \param obj name of the object
    #  \param distance distance of object to surface in preplacement
    #   configuration
    def setPreplacementDistance(self, obj, distance):
        # check that object is registered in factory
        if not obj in self.objects:
            raise RuntimeError(obj + " is not references as an object in the "
                               + "factory")
        self.preplaceDistance[obj] = distance

    ## Get preplacement distance
    #  \param obj name of the object
    def getPreplacementDistance(self, obj):
        # check that object is registered in factory
        if not obj in self.objects:
            raise RuntimeError(obj + " is not references as an object in the "
                               + "factory")
        return self.preplaceDistance.get(obj, self.defaultPreplaceDist)

    ## \}
