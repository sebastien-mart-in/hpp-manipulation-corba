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
from client import Client

class ProblemSolver (object):
    """
    This class wraps the Corba client to the server implemented by
    libhpp-manipulation-corba.so

    Some method implemented by the server can be considered as private. The
    goal of this class is to hide them and to expose those that can be
    considered as public.
    """
    def __init__ (self):
        self.client = Client ()

    def loadRobotModel (self, robotName, rootJointType, packageName, modelName,
                        urdfSuffix, srdfSuffix):

        """
        Load robot model and store in local map

        input robotName key of the robot in ProblemSolver object map
              (see hpp::manipulation::ProblemSolver::addRobot)
        input rootJointType type of root joint among "anchor", "freeflyer",
              "planar",
        input packageName Name of the ROS package containing the model,
        input modelName Name of the package containing the model
        input urdfSuffix suffix for urdf file,

        The ros url are built as follows:
        "package://${packageName}/urdf/${modelName}${urdfSuffix}.urdf"
        "package://${packageName}/srdf/${modelName}${srdfSuffix}.srdf"

        """
        self.client.robot.loadRobotModel (robotName, rootJointType,
                                          packageName, modelName,
                                          urdfSuffix, srdfSuffix)

    def loadHumanoidModel (self, robotName, rootJointType,
                           packageName, modelName,
                           urdfSuffix, srdfSuffix):
        """
        Load humanoid robot model and store in local map

        input robotName key of the robot in ProblemSolver object map
              (see hpp::manipulation::ProblemSolver::addRobot)
        input rootJointType type of root joint among "anchor", "freeflyer",
              "planar",
        input packageName Name of the ROS package containing the model,
        input modelName Name of the package containing the model
        input urdfSuffix suffix for urdf file,

        The ros url are built as follows:
        "package://${packageName}/urdf/${modelName}${urdfSuffix}.urdf"
        "package://${packageName}/srdf/${modelName}${srdfSuffix}.srdf"
        """
        self.client.robot.loadHumanoidModel (robotName, rootJointType,
                                             packageName, modelName,
                                             urdfSuffix, srdfSuffix)

    def loadObjectModel (self, objectName, rootJointType,
                         packageName, modelName,
                         urdfSuffix, srdfSuffix):
        """
        Load object model and store in local map

        input robotName key of the object in ProblemSolver object map
              (see hpp::manipulation::ProblemSolver::addRobot)
        input rootJointType type of root joint among "anchor", "freeflyer",
              "planar",
        input packageName Name of the ROS package containing the model,
        input modelName Name of the package containing the model
        input urdfSuffix suffix for urdf file,

        The ros url are built as follows:
        "package://${packageName}/urdf/${modelName}${urdfSuffix}.urdf"
        "package://${packageName}/srdf/${modelName}${srdfSuffix}.srdf"
        """
        self.client.robot.loadObjectModel (objectName, rootJointType,
                                           packageName, modelName,
                                           urdfSuffix, srdfSuffix)
        # Read srdf file for object specific informations (handles).
        # Build filename from ROS_PACKAGE_PATH
        import os
        ros_package_path = os.getenv ('ROS_PACKAGE_PATH').split(":")
        path = None
        for p in ros_package_path:
            if os.path.exists (p + '/' + packageName):
                path = p + '/' + packageName
                break
        if not path:
            raise IOError ("package " + packageName + " not found")
        filename = path + '/' + 'srdf/' + modelName + srdfSuffix + '.srdf'
        parser = SrdfParser (client, objectName, filename)
        parser.parse ()

    def buildCompositeRobot (self, robotName, robotNames):
        """
        Build a composite robot from a set of robots and objects

        input robotName Name of the composite robot,
        input robotNames list of names of the robots and objects.
        """
        self.client.robot.buildCompositeRobot (robotName, robotNames)
