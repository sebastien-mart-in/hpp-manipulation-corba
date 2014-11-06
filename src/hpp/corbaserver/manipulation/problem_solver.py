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

from srdf_parser import Parser as SrdfParser

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

    ## \brief Add goal configuration to specified problem.
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
    #  If joint1 of joint2 is "", the corresponding joint is replaced by
    #  the global frame.
    #  constraints are stored in ProblemSolver object
    def createOrientationConstraint (self, constraintName, joint1Name,
                                     joint2Name, p):
        return self.client.basic.problem.createOrientationConstraint \
            (constraintName, joint1Name, joint2Name, p)

    ## Create position constraint between two joints
    #
    #  \param constraintName name of the constraint created,
    #  \param joint1Name name of first joint
    #  \param joint2Name name of second joint
    #  \param point1 point in local frame of joint1,
    #  \param point2 point in local frame of joint2.
    #  If joint1 of joint2 is "", the corresponding joint is replaced by
    #  the global frame.
    #  constraints are stored in ProblemSolver object
    def createPositionConstraint (self, constraintName, joint1Name,
                                  joint2Name, point1, point2):
        return self.client.basic.problem.createPositionConstraint \
            (constraintName, joint1Name, joint2Name, point1, point2)

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
            self.createLockedDofConstraint (lockDofBname + '_'  + axis,\
                    freeflyerBname + '_xyz', values[rankcfg    ], rankcfg    , rankvel)
            self.createLockedDofConstraint (lockDofBname + '_r' + axis,\
                    freeflyerBname + '_SO3', values[rankcfg + 4], rankcfg + 1, rankvel)
            self.isLockedDofParametric (lockDofBname + '_'  + axis , parametric)
            self.isLockedDofParametric (lockDofBname + '_r' + axis , parametric)
            lockbox.append (lockDofBname + '_'  + axis)
            lockbox.append (lockDofBname + '_r' + axis)
            rankcfg = rankcfg + 1
            rankvel = rankvel + 1
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
    ## \}

    ## \name Solve problem and get paths
    # \{

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
    ## \}
