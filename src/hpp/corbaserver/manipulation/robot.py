#!/usr/bin/env python
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

from hpp.corbaserver.manipulation import Client as ManipulationClient
from hpp.corbaserver.wholebody_step import Client as WholebodyStepClient
from hpp.corbaserver import Client as BasicClient

from srdf_parser import Parser as SrdfParser

class CorbaClient:
    """
    Container for corba clients to various interfaces.
    """
    def __init__ (self):
        self.basic = BasicClient ()
        self.manipulation = ManipulationClient ()
        self.wholebodyStep = WholebodyStepClient ()

class Robot (object):
    """
    Helper class to enhance corba interface
    """
    def __init__ (self, robotName, rootJointType):
        self.robotName = robotName
        self.client = CorbaClient ()
        self.loadModel (robotName, rootJointType)

    def loadModel (self, robotName, rootJointType):
        """
        Virtual function to load robot model

          Overloaded in HumanoidRobot class
        """
        self.client.manipulation.robot.loadRobotModel \
            (robotName, rootJointType, self.packageName, self.urdfName,
             self.urdfSuffix, self.srdfSuffix)

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
        self.client.manipulation.robot.loadRobotModel (robotName, rootJointType,
                                                       packageName, modelName,
                                                       urdfSuffix, srdfSuffix)

    def loadHumanoidModel (self, robotName, rootJointType, packageName,
                           modelName, urdfSuffix, srdfSuffix):
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
        self.client.manipulation.robot.loadHumanoidModel \
            (robotName, rootJointType, packageName, modelName,
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
        self.client.manipulation.robot.loadObjectModel \
            (objectName, rootJointType, packageName, modelName, urdfSuffix,
             srdfSuffix)
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
        parser = SrdfParser (self.client.manipulation, objectName, filename)
        parser.parse ()

    def buildCompositeRobot (self, robotName, robotNames):
        """
        Build a composite robot from a set of robots and objects

        input robotName Name of the composite robot,
        input robotNames list of names of the robots and objects.
        """
        self.client.manipulation.robot.buildCompositeRobot (robotName,
                                                            robotNames)
        self.jointNames = self.client.basic.robot.getJointNames ()
        self.rankInConfiguration = dict ()
        self.rankInVelocity = dict ()
        rankInConfiguration = rankInVelocity = 0
        for j in self.jointNames:
            self.rankInConfiguration [j] = rankInConfiguration
            rankInConfiguration += \
                self.client.basic.robot.getJointConfigSize (j)
            self.rankInVelocity [j] = rankInVelocity
            rankInVelocity += self.client.basic.robot.getJointNumberDof (j)

    def setTranslationBounds (self, xmin, xmax, ymin, ymax, zmin, zmax):
        """
        Set bounds on the translation part of the freeflyer joint.

          Valid only if the robot has a freeflyer joint.
        """
        self.client.basic.robot.setJointBounds (self.robotName + "base_joint_x",
                                                [xmin, xmax])
        self.client.basic.robot.setJointBounds (self.robotName + "base_joint_y",
                                                [ymin, ymax])
        self.client.basic.robot.setJointBounds (self.robotName + "base_joint_z",
                                                [zmin, zmax])

class HumanoidRobot (Robot):

    def __init__ (self, robotName, rootJointType):
        Robot.__init__ (self, robotName, rootJointType)

    def loadModel (self, robotName, rootJointType):
        self.client.manipulation.robot.loadHumanoidModel \
            (robotName, rootJointType, self.packageName, self.urdfName,
             self.urdfSuffix, self.srdfSuffix)
