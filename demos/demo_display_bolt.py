#!/usr/bin/env python3
""" @namespace Basic loading and visualization for the Bolt robot using gepetto viewer.
@file display_bolt.py
@copyright Copyright (c) 2020,
           New York University and Max Planck Gesellschaft,
           License BSD-3-Clause
@example display_bolt.py
- 1. Building the workspace by executing `catkin build` in the workspace.
- 2. "source ./devel/setup.bash" is called from the root of the catkin
     workspace.
- 3. Run the demo by either:
    - 3.1. `python3 display_bolt.py`
    - 3.2. `cd /path/to/robot_properties_bolt/` ; `./demos/display_bolt.py`
"""

import numpy as np
import pinocchio as se3
import time
import os

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
    q[10] = -0.5  # Right side of quadruped
    q[7] = 0.5  # Left side of quadruped

    # Display the configuration in the viewer.
    robot.display(q)

    # Example of moving the robot forward and updating the display every time.
    for i in range(10):
        q[0] += 0.05
        robot.display(q)
        time.sleep(0.2)
