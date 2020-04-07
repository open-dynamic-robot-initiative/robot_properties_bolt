## @namespace robot_properties_bolt.pd
""" This module reads planner data from files and runs a PD
    controller.

    @file pd.py
    @copyright Copyright (c) 2020,
               New York University and Max Planck Gesellschaft,
               License BSD-3-Clause
"""

import numpy as np
import time
import os
import rospkg
import pybullet as p
import pinocchio as se3

from py_pinocchio_bullet.wrapper import PinBulletWrapper
from robot_properties_bolt.config import BoltConfig
from robot_properties_bolt.bolt_wrapper import BoltRobot

if __name__ == "__main__":
    # Create a robot instance. This initializes the simulator as well.
    robot = BoltRobot()

    torque = np.loadtxt("Torque.txt")
    pos = np.loadtxt("Position.txt")
    vel = np.loadtxt("Velocity.txt")

    # Reset the robot to some initial state.
    q0 = BoltConfig.initial_configuration
    dq0 = BoltConfig.initial_velocity
    robot.reset_state(q0, dq0)

    p.setTimeStep(0.001)
    kp = 20
    kd = 0.1
    posHistory = []
    # Run the simulator
    for i in range(len(pos)):
        deltaPos = pos[i] - np.squeeze(np.asarray(robot.get_state()[0][7:]))
        deltaVel = vel[i] - np.squeeze(np.asarray(robot.get_state()[1][6:]))
        posHistory.append(np.squeeze(np.asarray(robot.get_state()[0][7:])))
        robot.send_joint_command(kp * deltaPos + kd * deltaVel)# + torque[i])
        robot.forward_robot()

        # Step the simulator.
        p.stepSimulation()
        time.sleep(0.001) # You can sleep here if you want to slow down the replay

    q, dq = robot.get_state()
    active_eff, forces = robot.get_force()

    def plot():
        import matplotlib.pyplot as plt
        fig, ax = plt.subplots(1, 1)
        ax.plot(posHistory, label="posHistory")
        ax.grid()
        ax.legend()
        plt.show()

    plot()

