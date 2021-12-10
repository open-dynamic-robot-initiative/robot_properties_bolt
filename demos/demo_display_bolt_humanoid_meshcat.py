#!/usr/bin/env python3
""" @namespace Basic loading and visualization for the Bolt robot using meshcat.
@file demo_display_bolt_meshcat.py
@copyright Copyright (c) 2021,
           New York University and Max Planck Gesellschaft,
           License BSD-3-Clause
@example demo_display_bolt_meshcat.py
"""

import sys
import pinocchio as pin
import time
from robot_properties_bolt.config import BoltHumanoidConfig

if __name__ == "__main__":

    # Load the robot urdf.
    robot = BoltHumanoidConfig.buildRobotWrapper()

    viz = pin.visualize.MeshcatVisualizer(
        robot.model, robot.collision_model, robot.visual_model
    )

    try:
        viz.initViewer(open=True)
    except ImportError as err:
        print(
            "Error while initializing the viewer. ",
            "It seems you should install Python meshcat"
        )
        print(err)
        sys.exit(0)

    viz.loadViewerModel()
    # Create a first initial position for the robot. Both legs are bent inwards.
    q = BoltHumanoidConfig.initial_configuration

    # q[[4]] = 0.6
    # Turn the legs outside
    q[10] = -0.1  # Right leg
    q[7] = 0.1  # Left leg

    viz.display(q)

    # Example of moving the robot forward and updating the display every time.
    for i in range(500):
        q[0] += 0.002
        viz.display(q)
        time.sleep(0.01)
