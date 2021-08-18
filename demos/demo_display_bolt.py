#!/usr/bin/env python3
""" @namespace Basic loading and visualization for the Bolt robot using gepetto viewer.
@file demo_display_bolt.py
@copyright Copyright (c) 2021,
           New York University and Max Planck Gesellschaft,
           License BSD-3-Clause
@example demo_display_bolt.py
"""

import time
from robot_properties_bolt.config import BoltConfig


if __name__ == "__main__":
    # Load the robot urdf.
    robot = BoltConfig.buildRobotWrapper()

    # Setup the display (connection to gepetto viewer) and load the robot model.
    robot.initViewer(loadModel=True)

    # Create a first initial position for the robot. Both legs are bent inwards.
    q = BoltConfig.initial_configuration

    # q[[4]] = 0.6
    # Turn the legs outside
    q[10] = -0.5  # Right leg
    q[7] = 0.5  # Left leg

    # Display the configuration in the viewer.
    robot.display(q)

    # Example of moving the robot forward and updating the display every time.
    for i in range(10):
        q[0] += 0.05
        robot.display(q)
        time.sleep(0.2)
