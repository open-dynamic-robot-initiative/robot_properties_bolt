#!/usr/bin/env python3
"""demo_simulate_bolt

Basic loading and visualization for the Bolt robot using gepetto viewer.

License: BSD 3-Clause License
Copyright (C) 2018-2021, New York University , Max Planck Gesellschaft
Copyright note valid unless otherwise stated in individual files.
All rights reserved.

- 1. Building the workspace by executing `colcon build` in the workspace.
- 2. "source ./install/setup.bash" is called from the root of the catkin
     workspace.
- 3. Run the demo by: `python3 display_bolt.py`

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

