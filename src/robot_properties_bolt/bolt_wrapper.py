## @namespace robot_properties_bolt.bolt_wrapper
""" This module define the Bolt robot instance. This initializes
    the simulator as well.

    @copyright Copyright (c) 2020,
               New York University and Max Planck Gesellschaft,
               License BSD-3-Clause
"""

import numpy as np
import time
import os
import pybullet
from ament_index_python.packages import get_package_share_directory
from pinocchio_bullet.wrapper import PinBulletWrapper
from robot_properties_bolt.config import BoltConfig


dt = 1e-3

class BoltRobot(PinBulletWrapper):
    @staticmethod
    def initPhysicsClient():
        physicsClient = pybullet.connect(pybullet.GUI)
        pybullet.setGravity(0,0, -9.81)
        pybullet.setPhysicsEngineParameter(fixedTimeStep=dt, numSubSteps=1)
        return physicsClient

    def __init__(self, physicsClient=None, useFixedBase=False):
        if physicsClient is None:
            self.physicsClient = self.initPhysicsClient()

        # Load the plain.
        plain_urdf = (get_package_share_directory("robot_properties_bolt") +
                      "/urdf/plane_with_restitution.urdf")
        self.planeId = pybullet.loadURDF(plain_urdf, useFixedBase=True)

        # Load the robot
        robotStartPos = [0., 0, 0.40]
        robotStartOrientation = pybullet.getQuaternionFromEuler([0, 0, 0])

        self.urdf_path = BoltConfig.urdf_path
        self.robotId = pybullet.loadURDF(self.urdf_path, robotStartPos,
            robotStartOrientation, flags=pybullet.URDF_USE_INERTIA_FROM_FILE,
            useFixedBase=useFixedBase)
        pybullet.getBasePositionAndOrientation(self.robotId)

        # Create the robot wrapper in pinocchio.
        package_dirs = [os.path.dirname(os.path.dirname(self.urdf_path)) + '/urdf']
        self.pin_robot = BoltConfig.buildRobotWrapper()

        # Query all the joints.
        num_joints = pybullet.getNumJoints(self.robotId)

        for ji in range(num_joints):
            pybullet.changeDynamics(self.robotId, ji, linearDamping=.04,
                angularDamping=0.04, restitution=0.0, lateralFriction=0.5)

        self.base_link_name = "base_link"
        controlled_joints = []
        for leg in ['FL', 'FR']:
            controlled_joints += [leg + '_HAA', leg + '_HFE', leg + '_KFE']
        self.joint_names = controlled_joints
        self.end_effector_names = ['FL_ANKLE', 'FR_ANKLE']


        # Creates the wrapper by calling the super.__init__.
        super(BoltRobot, self).__init__(self.robotId, self.pin_robot,
            controlled_joints,
            self.end_effector_names
        )

    def forward_robot(self, q=None, dq=None):
        if q is None:
            q, dq = self.get_state()
        elif dq is None:
            raise ValueError('Need to provide q and dq or non of them.')

        self.pin_robot.forwardKinematics(q, dq)
        self.pin_robot.computeJointJacobians(q)
        self.pin_robot.framesForwardKinematics(q)
        self.pin_robot.centroidalMomentum(q, dq)
