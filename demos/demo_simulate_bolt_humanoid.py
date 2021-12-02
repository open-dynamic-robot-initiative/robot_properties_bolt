#!/usr/bin/env python
"""demo_simulate_bolt

Basic Simulation for the BoltHumanoid robot in Pysbullet.

License: BSD 3-Clause License
Copyright (C) 2018-2021, New York University , Max Planck Gesellschaft
Copyright note valid unless otherwise stated in individual files.
All rights reserved.

- 1. Building the workspace by executing `colcon build` in the workspace.
- 2. "source ./install/setup.bash" is called from the root of the catkin
     workspace.
- 3. Run the demo

"""

from math import sin
import numpy as np
from bullet_utils.env import BulletEnvWithGround
from robot_properties_bolt.bolt_humanoid_wrapper import BoltHumanoidRobot, BoltHumanoidConfig


if __name__ == "__main__":
    # Create a robot instance. This initializes the simulator as well.
    env = BulletEnvWithGround()
    robot = BoltHumanoidRobot(use_fixed_base=True)
    env.add_robot(robot)

    # Initialize the control torques.
    tau = np.zeros(robot.nb_dof)

    # Reset the robot to some initial state.
    q0 = BoltHumanoidConfig.initial_configuration
    q0.fill(0.0)
    q0[2] = 0.5
    q0[6] = 1.0
    dq0 = BoltHumanoidConfig.initial_velocity
    robot.reset_state(q0, dq0)

    # Run the simulator for 100 steps
    for i in range(50000):
        q, dq = robot.get_state()

        t = float(i) * 0.001
        kp = 5.0
        kd = 0.2

        q_target = np.array(BoltHumanoidConfig.initial_configuration[7:]) + 0.5 * sin(
            t * np.pi
        )

        tau = kp * (q_target - q[7:]) - kd * dq[6:]

        robot.send_joint_command(tau)

        # Step the simulator.
        # You can use sleep here if you want to slow down the replay
        env.step(sleep=True)

    # Read the final state and forces after the stepping.
    q, dq = robot.get_state()
    active_eff, forces = robot.get_force()
    print("q", q)
    print("dq", dq)
    print("active eff", active_eff)
    print("forces", forces)
