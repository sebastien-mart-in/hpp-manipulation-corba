#!/usr/bin/env python
#
# Copyright (c) 2014 CNRS
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

## Definition of a manipulation planning problem
#
#  This class wraps the Corba client to the server implemented by
#  libhpp-manipulation-corba.so
#
#  Some method implemented by the server can be considered as private. The
#  goal of this class is to hide them and to expose those that can be
#  considered as public.
class ProblemSolver (object):
    def __init__ (self, robot):
        self.client = robot.client
        self.robot = robot

    ## \name Initial and goal configurations
    # \{

    ## Set initial configuration of specified problem.
    #	\param dofArray Array of degrees of freedom
    #	\throw Error.
    def setInitialConfig (self, dofArray):
        return self.client.basic.problem.setInitialConfig (dofArray)

    ## Get initial configuration of specified problem.
    #	\return Array of degrees of freedom
    def getInitialConfig (self):
        return self.client.basic.problem.getInitialConfig ()

    ## Add goal configuration to specified problem.
    #	\param dofArray Array of degrees of freedom
    #	\throw Error.
    def addGoalConfig (self, dofArray):
        return self.client.basic.problem.addGoalConfig (dofArray)

    ## Get goal configurations of specified problem.
    #	\return Array of degrees of freedom
    def getGoalConfigs (self):
        return self.client.basic.problem.getGoalConfigs ()

    ## Reset goal configurations
    def resetGoalConfigs (self):
        return self.client.basic.problem.resetGoalConfigs ()
    ## \}

    ## \name Obstacles
    # \{

    ## Load obstacle from urdf file
    #  \param package Name of the package containing the model,
    #  \param filename name of the urdf file in the package
    #         (without suffix .urdf)
    #  \param prefix prefix added to object names in case the same file is
    #         loaded several times
    #
    #  The ros url is built as follows:
    #  "package://${package}/urdf/${filename}.urdf"
    #
    #  The kinematic structure of the urdf file is ignored. Only the geometric
    #  objects are loaded as obstacles.
    def loadObstacleFromUrdf (self, package, filename, prefix):
        return self.client.basic.obstacle.loadObstacleModel (package, filename,
                                                       prefix)

    ## Remove an obstacle from outer objects of a joint body
    #
    #  \param objectName name of the object to remove,
    #  \param jointName name of the joint owning the body,
    #  \param collision whether collision with object should be computed,
    #  \param distance whether distance to object should be computed.
    #  \throw Error.
    def removeObstacleFromJoint (self, objectName, jointName, collision,
                                 distance):
        return self.client.basic.obstacle.removeObstacleFromJoint \
            (objectName, jointName, collision, distance)

    ## Move an obstacle to a given configuration.
    #  \param objectName name of the polyhedron.
    #  \param cfg the configuration of the obstacle.
    #  \throw Error.
    #
    #  \note The obstacle is not added to local map
    #  impl::Obstacle::collisionListMap.
    #
    #  \note Build the collision entity of polyhedron for KCD.
    def moveObstacle (self, objectName, cfg):
        return self.client.basic.obstacle.moveObstacle (objectName, cfg)
    ## Get the position of an obstacle
    #
    #  \param objectName name of the polyhedron.
    #  \retval cfg Position of the obstacle.
    #  \throw Error.
    def getObstaclePosition (self, objectName):
        return self.client.basic.obstacle.getObstaclePosition (objectName)

    ## Get list of obstacles
    #
    #  \param collision whether to return obstacle for collision,
    #  \param distance whether to return obstacles for distance computation
    # \return list of obstacles
    def getObstacleNames (self, collision, distance):
        return self.client.basic.obstacle.getObstacleNames (collision, distance)

    ##\}

    ## \name Constraints
    #  \{

    ##  Create a grasp constraint for the composite robot
    #
    #   constraint is stored in C++ hpp::core::ProblemSolver local map
    #   of numerical constraints.
    #   \sa hpp::core::ProblemSolver::addNumericalConstraint
    #
    #   \param graspName key in the map of numerical constraints,
    #   \param jointName name of the joint that grasps "robot/joint",
    #   \param handleName name of the handle grasped "object/handle",
    #   \param handlePositioninJoint position of the handle in the joint frame.
    def createGrasp (self, graspName, gripperName, handleName) :
        return self.client.manipulation.problem.createGrasp \
            (graspName, gripperName, handleName)

    ##  Create a pre-grasp constraint for the composite robot
    #
    #   constraint is stored in C++ hpp::core::ProblemSolver local map
    #   of numerical constraints.
    #   \sa hpp::core::ProblemSolver::addNumericalConstraint
    #
    #   \param preGraspName key in the map of numerical constraints,
    #   \param jointName name of the joint that grasps "robot/joint",
    #   \param handleName name of the handle grasped "object/handle",
    #   \param handlePositioninJoint position of the handle in the joint frame.
    def createPreGrasp (self, preGraspName, gripperName, handleName) :
        return self.client.manipulation.problem.createPreGrasp \
            (preGraspName, gripperName, handleName)

    ##  Create static stability constraints
    #
    #   Call corba request
    #   hpp::corbaserver::wholebody_step::Problem::addStaticStabilityConstraints
    #
    #   The ankles are defined by members leftAnkle and rightAnkle of variable
    #   robot passed at construction of this object.
    #   \param constraintName name of the resulting constraint,
    #   \param q0 configuration that satisfies the constraints
    def createStaticStabilityConstraints (self, constraintName, q0):
        self.client.wholebodyStep.problem.addStaticStabilityConstraints \
            (constraintName, q0, self.robot.leftAnkle, self.robot.rightAnkle)
        self.balanceConstraints_ = [constraintName + "/relative-com",
                                    constraintName + "/relative-orientation",
                                    constraintName + "/relative-position",
                                    constraintName + "/orientation-left-foot",
                                    constraintName + "/position-left-foot"]

    ##  Create complement of static stability constraints
    #
    #   Call corba request
    #   hpp::corbaserver::wholebody_step::Problem::addComplementStaticStabilityConstraints
    #
    #   The ankles are defined by members leftAnkle and rightAnkle of variable
    #   robot passed at construction of this object.
    #   \param constraintName name of the resulting constraint,
    #   \param q0 configuration that satisfies the constraints
    def createComplementStaticStabilityConstraints (self, constraintName, q0):
        self.client.wholebodyStep.problem.addComplementStaticStabilityConstraints \
            (constraintName, q0, self.robot.leftAnkle, self.robot.rightAnkle)

    ## Return balance constraints created by method
    #  ProblemSolver.createStaticStabilityConstraints
    def balanceConstraints (self):
        return self.balanceConstraints_

    ## Reset Constraints
    #

    ## Create orientation constraint between two joints
    #
    #  \param constraintName name of the constraint created,
    #  \param joint1Name name of first joint
    #  \param joint2Name name of second joint
    #  \param p quaternion representing the desired orientation
    #         of joint2 in the frame of joint1.
    #  \param mask Select which axis to be constrained.
    #  If joint1 of joint2 is "", the corresponding joint is replaced by
    #  the global frame.
    #  constraints are stored in ProblemSolver object
    def createOrientationConstraint (self, constraintName, joint1Name,
                                     joint2Name, p, mask):
        return self.client.basic.problem.createOrientationConstraint \
            (constraintName, joint1Name, joint2Name, p, mask)

    ## Create position constraint between two joints
    #
    #  \param constraintName name of the constraint created,
    #  \param joint1Name name of first joint
    #  \param joint2Name name of second joint
    #  \param point1 point in local frame of joint1,
    #  \param point2 point in local frame of joint2.
    #  \param mask Select which axis to be constrained.
    #  If joint1 of joint2 is "", the corresponding joint is replaced by
    #  the global frame.
    #  constraints are stored in ProblemSolver object
    def createPositionConstraint (self, constraintName, joint1Name,
                                  joint2Name, point1, point2, mask):
        return self.client.basic.problem.createPositionConstraint \
            (constraintName, joint1Name, joint2Name, point1, point2, mask)

    ## Reset Constraints
    #
    #  Reset all constraints, including numerical constraints and locked
    #  degrees of freedom.
    def resetConstraints (self):
        return self.client.basic.problem.resetConstraints ()

    ## Set numerical constraints in ConfigProjector
    #
    #  \param name name of the resulting numerical constraint obtained
    #         by stacking elementary numerical constraints,
    #  \param names list of names of the numerical constraints as
    #         inserted by method hpp::core::ProblemSolver::addNumericalConstraint.
    def setNumericalConstraints (self, name, names):
        return self.client.basic.problem.setNumericalConstraints (name, names)

    ## Apply constraints
    #
    #  \param q initial configuration
    #  \return configuration projected in success,
    #  \throw Error if projection failed.
    def applyConstraints (self, q):
        return self.client.basic.problem.applyConstraints (q)

    ## Generate a configuration satisfying the constraints
    #
    #  \param maxIter maximum number of tries,
    #  \return configuration projected in success,
    #  \throw Error if projection failed.
    def generateValidConfig (self, maxIter):
        return self.client.basic.problem.generateValidConfig (maxIter)

    ## Insert a new LockedDof constraint with given value in the
    #        hpp::manipulation::ProblemSolver map
    # \param lockedDofName key of the constraint in the map
    # \param jointName name of the joint
    # \param value value of the locked degree of freedom,
    # \param rankInConfiguration rank of the locked dof in the joint
    #        configuration vector (should be 0 for rotation and translation
    #        joints)
    # \param rankInVelocity rank of the locked dof in the joint
    #        velocity vector (should be 0 for rotation and translation
    #        joints)
    def createLockedDofConstraint (self, lockedDofName, jointName, value, rankInConfiguration, rankInVelocity):
        return self.client.manipulation.problem.createLockedDofConstraint \
            (lockedDofName, jointName, value, rankInConfiguration, rankInVelocity)

    ## Make a LockedDof constraint parametric or non-parametric
    # \param constraintName Name of the numerical constraint,
    # \param value True is parametric
    #
    # Constraints should have been added in the ProblemSolver local map.
    def isLockedDofParametric (self, constraintName, parametric):
        self.client.manipulation.problem.isLockedDofParametric \
                (constraintName, parametric)

    ## Lock degree of freedom of a FreeFlyer joint
    # \param freeflyerBname base name of the joint (It will be completed by '_xyz' and '_SO3'),
    # \param lockDofBname base name of the lockdof constraints (It will be completed by '_r?[xyz]'),
    # \param values value of the locked degree of freedom as a quaternion,
    # \param parametric whether the lockDofs are parametric.
    def lockFreeFlyerJoint (self, freeflyerBname, lockDofBname, values = (0,0,0,1,0,0,0), parametric = False):
        rankcfg = 0; rankvel = 0; lockbox = list ()
        for axis in ['x','y','z']:
            namet = lockDofBname + '_'  + axis
            namer = lockDofBname + '_r' + axis
            self.createLockedDofConstraint (namet,\
                    freeflyerBname + '_xyz', values[rankcfg    ], rankcfg    , rankvel)
            self.createLockedDofConstraint (namer,\
                    freeflyerBname + '_SO3', values[rankcfg + 4], rankcfg + 1, rankvel)
            self.isLockedDofParametric (namet, parametric)
            self.isLockedDofParametric (namer, parametric)
            lockbox.append (namet)
            lockbox.append (namer)
            rankcfg = rankcfg + 1
            rankvel = rankvel + 1
        return lockbox

    ## Lock degree of freedom of a Planar joint
    # \param planarBname base name of the joint (It will be completed by '_xy' and '_rz'),
    # \param lockDofBname base name of the lockdof constraints (It will be completed by '(_rz)?_xy'),
    # \param values value of the locked degree of freedom as (t_x, t_y, cos (r_z), sin (r_z)),
    # \param parametric whether the lockDofs are parametric.
    def lockPlanarJoint (self, freeflyerBname, lockDofBname, values = (0,0,1,0), parametric = False):
        rank = 0; lockbox = list ()
        for axis in ['_x','_y']:
            namet = lockDofBname + axis
            namer = lockDofBname + '_rz' + axis
            self.createLockedDofConstraint (namet,\
                    freeflyerBname + '_xy', values[rank], rank, rank)
            self.createLockedDofConstraint (namer,\
                    freeflyerBname + '_rz', values[rank], rank, rank)
            self.isLockedDofParametric (namet, parametric)
            self.isLockedDofParametric (namer, parametric)
            lockbox.append (namet)
            lockbox.append (namer)
            rank = rank + 1
        return lockbox

    ## Lock degree of freedom with given value
    # \param jointName name of the joint
    # \param value value of the locked degree of freedom,
    # \param rankInConfiguration rank of the locked dof in the joint
    #        configuration vector
    # \param rankInVelocity rank of the locked dof in the joint
    #        velocity vector
    def lockDof (self, jointName, value, rankInConfiguration, rankInVelocity):
        return self.client.basic.problem.lockDof \
            (jointName, value, rankInConfiguration, rankInVelocity)

    ## Lock joint with one degree of freedom with given value
    # \param jointName name of the joint
    # \param value value of the locked degree of freedom,
    def lockOneDofJoint (self, jointName, value):
        return self.client.basic.problem.lockDof (jointName, value, 0, 0)

    ## error threshold in numerical constraint resolution
    def setErrorThreshold (self, threshold):
        return self.client.basic.problem.setErrorThreshold (threshold)

    ## Set the maximal number of iterations
    def setMaxIterations (self, iterations):
	return self.client.basic.problem.setMaxIterations (iterations)


    ## \}

    ## \name Solve problem and get paths
    # \{

    ## Select path planner type
    #  \param Name of the path planner type, either "DiffusingPlanner",
    #         "VisibilityPrmPlanner", or any type added by method
    #         core::ProblemSolver::addPathPlannerType
    def selectPathPlanner (self, pathPlannerType):
        return self.client.basic.problem.selectPathPlanner (pathPlannerType)

    ## Select path optimizer type
    #  \param Name of the path optimizer type, either "RandomShortcut" or
    #   any type added by core::ProblemSolver::addPathOptimizerType
    def selectPathOptimizer (self, pathOptimizerType):
        return self.client.basic.problem.selectPathOptimizer (pathOptimizerType)

    ## Select path validation method
    #  \param Name of the path validation method, either "Discretized"
    #  "Progressive", "Dichotomy", or any type added by
    #  core::ProblemSolver::addPathValidationType,
    #  \param tolerance maximal acceptable penetration.
    def selectPathValidation (self, pathValidationType, tolerance):
        return self.client.basic.problem.selectPathValidation \
            (pathValidationType, tolerance)

    ## Solve the problem of corresponding ChppPlanner object
    def solve (self):
        return self.client.basic.problem.solve ()

    ## Make direct connection between two configurations
    #  \param startConfig, endConfig: the configurations to link.
    #  \throw Error if steering method fails to create a direct path of if
    #  direct path is not valid
    def directPath (self, startConfig, endConfig):
        return self.client.basic.problem.directPath (startConfig, endConfig)

    ## Get Number of paths
    def numberPaths (self):
        return self.client.basic.problem.numberPaths ()

    ## Optimize a given path
    # \param inPathId Id of the path in this problem.
    # \throw Error.
    def optimizePath(self, inPathId):
        return self.client.basic.problem.optimizePath (inPathId)

    ## Get length of path
    # \param inPathId rank of the path in the problem
    # \return length of path if path exists.
    def pathLength(self, inPathId):
        return self.client.basic.problem.pathLength(inPathId)

    ## Get the robot's config at param on the a path
    # \param inPathId rank of the path in the problem
    # \param atDistance : the user parameter choice
    # \return dofseq : the config at param
    def configAtDistance(self, inPathId, atDistance):
        return self.client.basic.problem.configAtDistance(inPathId, atDistance)

    ## Get way points of a path
    #  \param pathId rank of the path in the problem
    def getWaypoints (self, pathId):
        return self.client.basic.problem.getWaypoints (pathId)

    ## \name Interruption of a path planning request
    #  \{

    ## \brief Interrupt path planning activity
    #   \note this method is effective only when multi-thread policy is used
    #         by CORBA server.
    #         See constructor of class Server for details.
    def interruptPathPlanning (self):
        return self.client.basic.problem.interruptPathPlanning ()
    # \}

    ## \name exploring the roadmap
    #  \{

    ## Get nodes of the roadmap.
    def nodes(self):
	return self.client.basic.problem.nodes ()

    ## Number of edges
    def numberEdges (self):
        return self.client.basic.problem.numberEdges ()

    ## Edge at given rank
    def edge (self, edgeId):
        return self.client.basic.problem.edge (edgeId)

    ## Number of connected components
    def numberConnectedComponents (self):
        return self.client.basic.problem.numberConnectedComponents ()

    ## Nodes of a connected component
    #  \param connectedComponentId index of connected component in roadmap
    #  \return list of nodes of the connected component.
    def nodesConnectedComponent (self, ccId):
        return self.client.basic.problem.nodesConnectedComponent (ccId)

    ## Clear the roadmap
    def clearRoadmap (self):
        return self.client.basic.problem.clearRoadmap ()
    ## \}
