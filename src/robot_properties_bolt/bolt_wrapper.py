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
from bullet_utils.wrapper import PinBulletWrapper
from robot_properties_bolt.config import BoltConfig
from robot_properties_solo.utils import find_paths


dt = 1e-3

class BoltRobot(PinBulletWrapper):

    def __init__(self, pos=None, orn=None):

        # Load the robot
        if pos is None:
            pos = [0.0, 0, 0.40]
        if orn is None:
            orn = pybullet.getQuaternionFromEuler([0, 0, 0])

        pybullet.setAdditionalSearchPath(BoltConfig.paths["package"])
        self.urdf_path = BoltConfig.urdf_path
        self.robotId = pybullet.loadURDF(
            self.urdf_path,
            pos, orn,
            flags=pybullet.URDF_USE_INERTIA_FROM_FILE,
            useFixedBase=False,
        )
        
        self.pin_robot = BoltConfig.buildRobotWrapper()

        # Query all the joints.
        num_joints = pybullet.getNumJoints(self.robotId)

        for ji in range(num_joints):
            pybullet.changeDynamics(self.robotId, ji, linearDamping=.04,
                angularDamping=0.04, restitution=0.0, lateralFriction=0.5)

        self.base_link_name = "base_link"
        self.end_eff_ids = []
        self.end_effector_names = []
        controlled_joints = []
        for leg in ['FL', 'FR']:
            controlled_joints += [leg + '_HAA', leg + '_HFE', leg + '_KFE']
            self.end_eff_ids.append(self.pin_robot.model.getFrameId(leg + "_ANKLE"))
            self.end_effector_names.append(leg + "_ANKLE")

        self.joint_names = controlled_joints
        self.nb_ee = len(self.end_effector_names)

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
