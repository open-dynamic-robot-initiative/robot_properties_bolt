""" Basic loading and visualization for the Bolt robot using gepetto viewer. """

import numpy as np
import pinocchio as se3
import time
import os

from robot_properties_bolt.config import BoltConfig

# Load the robot urdf.
robot = BoltConfig.buildRobotWrapper()

# Setup the display (connection to gepetto viewer) and load the robot model.
robot.initDisplay(loadModel=True)

# Create a first initial position for the robot. Both legs are bent inwards.
q = np.matrix(BoltConfig.initial_configuration).T

# q[[4]] = 0.6
# Turn the legs outside
q[[10]] = -0.5  # Right side of quadruped
q[[7]] = 0.5  # Left side of quadruped

# Display the configuration in the viewer.
robot.display(q)

# Example of moving the robot forward and updating the display every time.
for i in range(10):
    q[0] += 0.05
    robot.display(q)
    time.sleep(0.2)
