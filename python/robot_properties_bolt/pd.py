import numpy as np

import time

import os
import rospkg
import pybullet as p
import pinocchio as se3

from py_pinocchio_bullet.wrapper import PinBulletWrapper
from robot_properties_bolt.config import boltConfig
from robot_properties_bolt.boltwrapper import BoltRobot

if __name__ == "__main__":
    # Create a robot instance. This initializes the simulator as well.
    robot = BoltRobot()
    tau = np.loadtxt("Torque.txt")
    pos = np.loadtxt("Position.txt")
    vel = np.loadtxt("Velocity.txt")

    # Reset the robot to some initial state.
    q0 = np.matrix(boltConfig.initial_configuration).T
    dq0 = np.matrix(boltConfig.initial_velocity).T
    robot.reset_state(q0, dq0)

    p.setTimeStep(0.001)
    # Run the simulator
    kp = 20
    kd = 0.1
    posHistory = []
    test = []
    for i in range(len(pos)):
        deltapos = pos[i] - np.squeeze(np.asarray(robot.get_state()[0][7:]))
        deltavel = vel[i] - np.squeeze(np.asarray(robot.get_state()[1][6:]))
        posHistory.append(np.squeeze(np.asarray(robot.get_state()[0][7:])))
        test.append(pos[i])
        robot.send_joint_command(kp * deltapos + kd * deltavel)# + tau[i])
        robot.forward_robot()

        # Step the simulator.
        p.stepSimulation()
        time.sleep(0.001) # You can sleep here if you want to slow down the replay

    q, dq = robot.get_state()
    active_eff, forces = robot.get_force()

    def plot():
        import matplotlib.pyplot as plt
        fig, ax = plt.subplots(1, 1, figsize=(8, 8))
        ax.plot(posHistory, label="posHistory")
        ax.plot(test, label="pos")
        ax.grid()
        ax.legend()
        plt.show()

    plot()

