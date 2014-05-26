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

## Corba clients to the various servers
#
class CorbaClient:
    """
    Container for corba clients to various interfaces.
    """
    def __init__ (self):
        self.basic = BasicClient ()
        self.manipulation = ManipulationClient ()
        self.wholebodyStep = WholebodyStepClient ()

## Load and handle a composite robot for manipulation planning
#
#  A composite robot is a kinematic chain composed of several sub-kinematic
#  chains rooted at an anchor joint.
class Robot (object):
    def __init__ (self, robotName, rootJointType):
        self.rootJointType = rootJointType
        self.robotName = robotName
        self.client = CorbaClient ()
        self.loadModel (robotName, rootJointType)

    ## Virtual function to load the robot model
    def loadModel (self, robotName, rootJointType):
        self.client.manipulation.robot.loadRobotModel \
            (robotName, rootJointType, self.packageName, self.urdfName,
             self.urdfSuffix, self.srdfSuffix)

    ## Load robot model and store in local map
    #
    #  \param robotName key of the robot in manipulation::ProblemSolver object
    #         map (see manipulation::ProblemSolver::addRobot)
    #  \param rootJointType type of root joint among "anchor", "freeflyer",
    #         "planar",
    #  \param packageName Name of the ROS package containing the model,
    #  \param modelName Name of the package containing the model
    #  \param urdfSuffix suffix for urdf file,
    #
    #  The ros url are built as follows:
    #  \li "package://${packageName}/urdf/${modelName}${urdfSuffix}.urdf"
    #  \li "package://${packageName}/srdf/${modelName}${srdfSuffix}.srdf"
    def loadRobotModel (self, robotName, rootJointType, packageName, modelName,
                        urdfSuffix, srdfSuffix):
        self.client.manipulation.robot.loadRobotModel (robotName, rootJointType,
                                                       packageName, modelName,
                                                       urdfSuffix, srdfSuffix)

    ## Load humanoid robot model and store in local map
    #
    #  \param robotName key of the robot in ProblemSolver object map
    #         (see manipulation::ProblemSolver::addRobot)
    #  \param rootJointType type of root joint among "anchor", "freeflyer",
    #         "planar",
    #  \param packageName Name of the ROS package containing the model,
    #  \param modelName Name of the package containing the model
    #  \param urdfSuffix suffix for urdf file,
    #
    #  The ros url are built as follows:
    #  \li "package://${packageName}/urdf/${modelName}${urdfSuffix}.urdf"
    #  \li "package://${packageName}/srdf/${modelName}${srdfSuffix}.srdf"
    def loadHumanoidModel (self, robotName, rootJointType, packageName,
                           modelName, urdfSuffix, srdfSuffix):
        self.client.manipulation.robot.loadHumanoidModel \
            (robotName, rootJointType, packageName, modelName,
             urdfSuffix, srdfSuffix)

    ## Load object model and store in local map
    #
    #  \param robotName key of the object in ProblemSolver object map
    #         (see manipulation::ProblemSolver::addRobot)
    #  \param rootJointType type of root joint among "anchor", "freeflyer",
    #         "planar",
    #  \param packageName Name of the ROS package containing the model,
    #  \param modelName Name of the package containing the model
    #  \param urdfSuffix suffix for urdf file,
    #
    #  The ros url are built as follows:
    #  \li "package://${packageName}/urdf/${modelName}${urdfSuffix}.urdf"
    #  \li "package://${packageName}/srdf/${modelName}${srdfSuffix}.srdf"
    def loadObjectModel (self, objectName, rootJointType,
                         packageName, modelName,
                         urdfSuffix, srdfSuffix):
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

    ## Build a composite robot from a set of robots and objects
    #
    #  \param robotName Name of the composite robot,
    #  \param robotNames list of names of the robots and objects.
    def buildCompositeRobot (self, robotName, robotNames):
        self.client.manipulation.robot.buildCompositeRobot (robotName,
                                                            robotNames)
        self.jointNames = self.client.basic.robot.getJointNames ()
        self.__setattr__ (self.robotName + 'JointNames',
                          map (lambda n : n [len(self.robotName)+1:],
                               filter (lambda n: n[:len (self.robotName)]
                                       == self.robotName, self.jointNames)))
        self.rankInConfiguration = dict ()
        self.rankInVelocity = dict ()
        rankInConfiguration = rankInVelocity = 0
        for j in self.jointNames:
            self.rankInConfiguration [j] = rankInConfiguration
            rankInConfiguration += \
                self.client.basic.robot.getJointConfigSize (j)
            self.rankInVelocity [j] = rankInVelocity
            rankInVelocity += self.client.basic.robot.getJointNumberDof (j)

    ## \name Degrees of freedom
    #  \{

    ## Get size of configuration
    # \return size of configuration
    def getConfigSize (self):
        return self.client.basic.robot.getConfigSize ()

    # Get size of velocity
    # \return size of velocity
    def getNumberDof (self):
        return self.client.basic.robot.getNumberDof ()
    ## \}

    ## \name Joints
    #\{

    # Get joint names in the same order as in the configuration.
    def getJointNames (self):
        return self.client.basic.robot.getJointNames ()

    # Get joint position.
    def getJointPosition (self, jointName):
        return self.client.basic.robot.getJointPosition (jointName)

    # Get joint number degrees of freedom.
    def getJointNumberDof (self, jointName):
        return self.client.basic.robot.getJointNumberDof (jointName)

    # Get joint number config size.
    def getJointConfigSize (self, jointName):
        return self.client.basic.robot.getJointConfigSize (jointName)

    # set bounds for the joint
    def setJointBounds (self, jointName, inJointBound):
        return self.client.basic.robot.setJointBounds (jointName, inJointBound)

    ## Set bounds on the translation part of the freeflyer joint.
    #
    #  Valid only if the robot has a freeflyer joint.
    def setTranslationBounds (self, xmin, xmax, ymin, ymax, zmin, zmax):
        self.client.basic.robot.setJointBounds (self.robotName + "base_joint_x",
                                                [xmin, xmax])
        self.client.basic.robot.setJointBounds (self.robotName + "base_joint_y",
                                                [ymin, ymax])
        self.client.basic.robot.setJointBounds (self.robotName + "base_joint_z",
                                                [zmin, zmax])
    ## \}

    ## \name Access to current configuration
    #\{

    ## Set current configuration of composite robot
    #
    #  \param q configuration of the composite robot
    def setCurrentConfig (self, q):
        self.client.basic.robot.setCurrentConfig (q)

    ## Get current configuration of composite robot
    #
    #  \return configuration of the composite robot
    def getCurrentConfig (self):
        return self.client.basic.robot.getCurrentConfig ()
    ## \}

    ## \name Collision checking and distance computation
    # \{

    ## Test collision with obstacles and auto-collision.
    #
    # Check whether current configuration of robot is valid by calling
    # CkwsDevice::collisionTest ().
    # \return whether configuration is valid
    def collisionTest (self):
        return self.client.basic.robot.collisionTest ()

    ## Compute distances between bodies and obstacles
    #
    # \return list of distances,
    # \return names of the objects belonging to a body
    # \return names of the objects tested with inner objects,
    # \return  closest points on the body,
    # \return  closest points on the obstacles
    # \note outer objects for a body can also be inner objects of another
    # body.
    def distancesToCollision (self):
        return self.client.basic.robot.distancesToCollision ()
    ## \}

    ## \}
    ## \name Mass and inertia
    # \{

    ## Get mass of robot
    def getMass ():
        return self.client.robot.getMass ()

    ## Get position of center of mass
    def getCenterOfMass ():
        return self.client.robot.getCenterOfMass ()
    ## Get Jacobian of the center of mass
    def getJacobianCenterOfMass ():
        return self.client.robot.getJacobianCenterOfMass ()
    ##\}

class HumanoidRobot (Robot):

    def __init__ (self, robotName, rootJointType):
        Robot.__init__ (self, robotName, rootJointType)

    def loadModel (self, robotName, rootJointType):
        self.client.manipulation.robot.loadHumanoidModel \
            (robotName, rootJointType, self.packageName, self.urdfName,
             self.urdfSuffix, self.srdfSuffix)
