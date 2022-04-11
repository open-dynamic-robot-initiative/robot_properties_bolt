# @namespace robot_properties_bolt.bolt_wrapper
""" This module define the Bolt robot instance. This initializes
    the simulator as well.

    @copyright Copyright (c) 2020,
               New York University and Max Planck Gesellschaft,
               License BSD-3-Clause
"""

import numpy as np
import pybullet
from bullet_utils.wrapper import PinBulletWrapper
from robot_properties_bolt.config import BoltConfig


class BoltRobot(PinBulletWrapper):
    def __init__(
        self,
        pos=None,
        orn=None,
        init_sliders_pose=4 * [0],
        use_fixed_base=False,
        is_passive=False
    ):
        self.is_passive=is_passive

        # Load the robot
        if pos is None:
            pos = [0.0, 0, 0.40]
        if orn is None:
            orn = pybullet.getQuaternionFromEuler([0, 0, 0])

        pybullet.setAdditionalSearchPath(BoltConfig.resources.package_path)
        self.urdf_path = BoltConfig.urdf_path
        self.simu_urdf_path = BoltConfig.simu_urdf_path
        if self.is_passive:
            self.robotId = pybullet.loadURDF(
                self.simu_urdf_path,
                pos,
                orn,
                flags=pybullet.URDF_USE_INERTIA_FROM_FILE,
                useFixedBase=use_fixed_base,
            )
        else:
            self.robotId = pybullet.loadURDF(
                self.urdf_path,
                pos,
                orn,
                flags=pybullet.URDF_USE_INERTIA_FROM_FILE,
                useFixedBase=use_fixed_base,
            )

        self.pin_robot = BoltConfig.buildRobotWrapper()
        if self.is_passive:
            self.simu_pin_robot = BoltConfig.buildSimuRobotWrapper()

        # Query all the joints.
        num_joints = pybullet.getNumJoints(self.robotId)

        for ji in range(num_joints):
            pybullet.changeDynamics(
                self.robotId,
                ji,
                linearDamping=0.04,
                angularDamping=0.04,
                restitution=0.0,
                lateralFriction=0.5,
            )

        self.slider_a = pybullet.addUserDebugParameter(
            "a", 0, 1, init_sliders_pose[0]
        )
        self.slider_b = pybullet.addUserDebugParameter(
            "b", 0, 1, init_sliders_pose[1]
        )
        self.slider_c = pybullet.addUserDebugParameter(
            "c", 0, 1, init_sliders_pose[2]
        )
        self.slider_d = pybullet.addUserDebugParameter(
            "d", 0, 1, init_sliders_pose[3]
        )

        self.base_link_name = "base_link"
        self.end_eff_ids = []
        self.end_effector_names = []
        controlled_joints = []
        for leg in ["FL", "FR"]:
            if self.is_passive:
                controlled_joints += [
                    leg + "_HAA",
                    leg + "_HFE",
                    leg + "_KFE",
                    leg + "_ANKLE",
                ]
            else:
                controlled_joints += [
                    leg + "_HAA",
                    leg + "_HFE",
                    leg + "_KFE",
                ]
            self.end_eff_ids.append(
                self.pin_robot.model.getFrameId(leg + "_ANKLE")
            )
            self.end_effector_names.append(leg + "_ANKLE")

        self.joint_names = controlled_joints
        self.nb_ee = len(self.end_effector_names)

        # Creates the wrapper by calling the super.__init__.
        super(BoltRobot, self).__init__(
            self.robotId,
            self.pin_robot,
            controlled_joints,
            self.end_effector_names,
        )

    def get_state(self):
        # Returns a pinocchio like representation of the q, dq matrixes

        if self.is_passive:
            q_simu, dq_simu = super(BoltRobot, self).get_state()
            q = np.concatenate([q_simu[0:10], q_simu[11:14]])
            dq = np.concatenate([dq_simu[0:9], dq_simu[10:13]])
            return q, dq
        else:
            q, dq = super(BoltRobot, self).get_state()
            return q, dq

    def _get_state_passive_ankle(self):
        # Returns a pinocchio like representation of the q, dq matrixes
        return super(BoltRobot, self).get_state()

    def reset_state(self, q, dq):
        if self.is_passive:
            q_simu = np.concatenate([q[0:10], [0.0], q[10:13], [0.0]])
            dq_simu = np.concatenate([dq[0:9], [0.0], dq[9:12], [0.0]])
            super(BoltRobot, self).reset_state(q_simu, dq_simu)
        else:
            super(BoltRobot, self).reset_state(q, dq)


    def send_joint_command(self, tau):
        if self.is_passive:
            tau_simu = np.concatenate([tau[0:3], [0.0], tau[3:6], [0.0]])
            super(BoltRobot, self).send_joint_command(tau_simu)
        else:
            super(BoltRobot, self).send_joint_command(tau)

    def get_slider_position(self, letter):
        if letter == "a":
            return pybullet.readUserDebugParameter(self.slider_a)
        if letter == "b":
            return pybullet.readUserDebugParameter(self.slider_b)
        if letter == "c":
            return pybullet.readUserDebugParameter(self.slider_c)
        if letter == "d":
            return pybullet.readUserDebugParameter(self.slider_d)

    def forward_robot(self, q=None, dq=None):
        if q is None and dq is None:
            q, dq = self.get_state()
            q_simu, dq_simu = self._get_state_passive_ankle()
        elif q is not None or dq is not None:
            raise ValueError("Need to provide q and dq or none of them.")
        else:
            q_simu = np.concatenate([q[0:10], [0.0], q[10:13], [0.0]])
            dq_simu = np.concatenate([dq[0:9], [0.0], dq[9:12], [0.0]])

        self.pin_robot.forwardKinematics(q, dq)
        self.pin_robot.computeJointJacobians(q)
        self.pin_robot.framesForwardKinematics(q)
        self.pin_robot.centroidalMomentum(q, dq)

        self.simu_pin_robot.forwardKinematics(q_simu, dq_simu)
        self.simu_pin_robot.computeJointJacobians(q)
        self.simu_pin_robot.framesForwardKinematics(q)
        self.simu_pin_robot.centroidalMomentum(q, dq)
