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
    """
    This class wraps the Corba client to the server implemented by
    libhpp-manipulation-corba.so

    Some method implemented by the server can be considered as private. The
    goal of this class is to hide them and to expose those that can be
    considered as public.
    """
    def __init__ (self, robot):
        self.client = robot.client
        self.robot = robot

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
    def createGrasp (self, graspName, jointName, handleName,
                     handlePositioninJoint) :
        self.client.manipulation.problem.createGrasp \
            (graspName, jointName, handleName, handlePositioninJoint)

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

    ## Return balance constraints created by method
    #  ProblemSolver.createStaticStabilityConstraints
    def balanceConstraints (self):
        return self.balanceConstraints_
