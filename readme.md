Robot Properties Bolt
---------------------

URDF and Pinocchio integration of the Bolt robot.

### Installation

#### Standard dependencies

This package is based on [ROS2](https://docs.ros.org/) based on dashing release and [Pinocchio](https://stack-of-tasks.github.io/pinocchio/).

To install Pinocchio, the simplest way is using [Conda](https://docs.conda.io/en/latest/):

    ```
    conda install -c conda-forge pinocchio
    ```

Alternatively you can use any of the ways listed [here](https://stack-of-tasks.github.io/pinocchio/download.html).

#### Download the package

Use the [treep_machines_in_motion](https://github.com/machines-in-motion/treep_machines_in_motion) configuration.

Do not forget to **register your ssh public key in your settings**:

In short:
  ```
  mkdir -p ~/devel
  pip install treep
  cd ~/devel
  git clone git@github.com:machines-in-motion/treep_machines_in_motion.git
  treep --clone ROBOT_PROPERTIES_BOLT
  ```


### Usage

#### Demos/Examples

Below are some examples. 

**Loading Bolt in PyBullet**

```
import pybullet as p
from bullet_utils.env import BulletEnvWithGround
from robot_properties_bolt.bolt_wrapper import BoltRobot

env = BulletEnvWithGround(p.GUI)
robot = env.add_robot(BoltRobot)
```

**Run simulation on Max-Planck Institute cluster**

```
conda create -n bolt python=3.7
source activate bolt
conda install -c conda-forge pinocchio 

git clone git@github.com:machines-in-motion/bullet_utils.git
cd bullet_utils
pip3 install .

git clone git@github.com:open-dynamic-robot-initiative/robot_properties_bolt.git
cd robot_properties_bolt
pip3 install .
```
You find demos of Bolt on [gepetto-viewer](https://github.com/Gepetto/gepetto-viewer), [Meshcat](https://github.com/rdeits/meshcat-python) and [PyBullet](https://pybullet.org/wordpress/) in the`demos/` folder. 


### License and Copyrights

License BSD-3-Clause
Copyright (c) 2019, New York University and Max Planck Gesellschaft.


