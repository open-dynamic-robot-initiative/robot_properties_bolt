# @namespace robot_properties_bolt.config
""" This module includes configuration for the Bolt.

    @file config.py
    @copyright Copyright (c) 2020,
               New York University and Max Planck Gesellschaft,
               License BSD-3-Clause
"""

import numpy as np
from math import pi
from ament_index_python.packages import get_package_share_directory
from os.path import join, dirname
import pinocchio as se3
from pinocchio.utils import zero
from pinocchio.robot_wrapper import RobotWrapper


class BoltAbstract(object):
    """ Abstract class used for all Bolt robots. """

    # PID gains
    kp = 5.0
    kd = 0.1
    ki = 0.0

    # The Kt constant of the motor [Nm/A]: tau = I * Kt.
    motor_torque_constant = 0.025

    # Control time period.
    control_period = 0.001
    dt = control_period

    # MaxCurrent = 12 # Ampers
    max_current = 2

    # Maximum torques.
    max_torque = motor_torque_constant * max_current

    # Maximum control one can send, here the control is the current.
    max_control = max_current

    # ctrl_manager_current_to_control_gain I am not sure what it does so 1.0.
    ctrl_manager_current_to_control_gain = 1.0

    max_qref = pi

    base_link_name = 'base_link'
    end_effector_names = ['FL_ANKLE', 'FR_ANKLE']

    @classmethod
    def buildRobotWrapper(cls):
        # Rebuild the robot wrapper instead of using the existing model to
        # also load the visuals.
        robot = RobotWrapper.BuildFromURDF(
            cls.urdf_path, cls.meshes_path, se3.JointModelFreeFlyer())
        robot.model.rotorInertia[6:] = cls.motor_inertia
        robot.model.rotorGearRatio[6:] = cls.motor_gear_ration
        return robot

    def joint_name_in_single_string(self):
        joint_names = ""
        for name in self.robot_model.names[2:]:
            joint_names += name + " "
        return joint_names


class BoltConfig(BoltAbstract):
    robot_family = "bolt"
    robot_name = "bolt"

    # Here we use the same urdf as for the quadruped but without the freeflyer.
    urdf_path = (
        join(get_package_share_directory("robot_properties_" + robot_family),
             "urdf",
             robot_name + ".urdf")
    )

    meshes_path = [
        dirname(get_package_share_directory("robot_properties_" + robot_family))
    ]

    yaml_path = (
        join(get_package_share_directory("robot_properties_" + robot_family),
             "config",
             "dgm_parameters_bolt.yaml")
    )

    # The inertia of a single blmc_motor.
    motor_inertia = 0.0000045

    # The motor gear ratio.
    motor_gear_ration = 9.

    # pinocchio model.
    robot_model = se3.buildModelFromUrdf(urdf_path,
                                         se3.JointModelFreeFlyer())
    robot_model.rotorInertia[6:] = motor_inertia
    robot_model.rotorGearRatio[6:] = motor_gear_ration

    mass = np.sum([i.mass for i in robot_model.inertias])

    base_name = robot_model.frames[2].name

    # The number of motors, here they are the same as there are only revolute
    # joints.
    nb_joints = robot_model.nv - 6

    joint_names = ['FL_HAA', 'FL_HFE', 'FL_KFE', 'FR_HAA', 'FR_HFE', 'FR_KFE']

    # Mapping between the ctrl vector in the device and the urdf indexes.
    urdf_to_dgm = tuple(range(6))

    map_joint_name_to_id = {}
    map_joint_limits = {}
    for i, (name, lb, ub) in enumerate(zip(robot_model.names[1:],
                                           robot_model.lowerPositionLimit,
                                           robot_model.upperPositionLimit)):
        map_joint_name_to_id[name] = i
        map_joint_limits[i] = [float(lb), float(ub)]

    # Define the initial state.
    initial_configuration = np.array(
        [0., 0., 0.26487417, 0., 0., 0., 1.,
         -0.35, 0.78539816, -1.57079633, 0.35, 0.78539816, -1.57079633])

    #[0.2, 0., 0.2, 0., 0., 0., 1.] + 2*[0., 0.8, -1.6]
    initial_velocity = (6 + 6)*[0, ]

    q0 = np.zeros(robot_model.nq)
    q0[:] = initial_configuration
    v0 = np.zeros(robot_model.nv)
    a0 = np.zeros(robot_model.nv)
