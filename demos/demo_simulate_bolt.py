#!/usr/bin/env python3
"""demo_simulate_bolt

Basic loading and visualization for the Bolt robot using gepetto viewer.

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

import time
import numpy as np
from bullet_utils.env import BulletEnvWithGround
from robot_properties_bolt.bolt_wrapper import BoltRobot, BoltConfig
import pybullet as p


if __name__ == "__main__":
    # Create a robot instance. This initializes the simulator as well.
    env = BulletEnvWithGround(p.GUI)
    robot = env.add_robot(BoltRobot)
    tau = np.zeros(robot.nb_dof)

    # Reset the robot to some initial state.
    q0 = BoltConfig.initial_configuration
    dq0 = BoltConfig.initial_velocity
    robot.reset_state(q0, dq0)

    # Run the simulator for 100 steps
    for i in range(500):
        # TODO: Implement a controller here.
        robot.send_joint_command(tau)

        # Step the simulator.
        env.step(sleep=True) # You can use sleep here if you want to slow down the replay

    # Read the final state and forces after the stepping.
    q, dq = robot.get_state()
    active_eff, forces = robot.get_force()
    print('q', q)
    print('dq', dq)
    print('active eff', active_eff)
    print('forces', forces)

