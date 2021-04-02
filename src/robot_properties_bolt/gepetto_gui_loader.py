## @namespace robot_properties_bolt.gepetto_gui_loader
""" This module shows Bolt in gepetto_gui.

    @file gepetto_gui_loader.py
    @copyright Copyright (c) 2020,
               New York University and Max Planck Gesellschaft,
               License BSD-3-Clause
"""


import eigenpy

eigenpy.switchToNumpyMatrix()
import time
from config import BoltConfig
from py_gepetto_gui_helper.gepetto_gui_scene import GepettoGuiScene
from py_gepetto_gui_helper.robot_visual import RobotVisual
from py_gepetto_gui_helper.frame import Frame


def create_scene():
    """
    Just create a scene for the bolt to be in
    """
    return GepettoGuiScene("bolt_scene", "bolt_window")


def load_bolt_in_gepetto_gui(gepetto_scene, robot_name):
    """
    Load the bolt meshes in the scene
    """
    config = BoltConfig()
    return RobotVisual(
        gepetto_scene, robot_name, config.urdf_path, config.meshes_path
    )


def display_bolt_in_gepetto_gui(launch_gepetto_gui_exec=False):
    """
    Uses the function above to load the urdf model of bolt in gepetto gui
    and load it in the initial configuration
    """

    if launch_gepetto_gui_exec:
        # create a new window
        gepetto_gui_process = GepettoGuiScene.open_gepetto_gui()

    # create a scene in it
    gui_scene = create_scene()
    # load the robot
    bolt_visual = load_bolt_in_gepetto_gui(gui_scene, "bolt")
    # place the robot in initial configuration
    config = BoltConfig()
    bolt_visual.display(config.q0)
    # place the world frame
    world_frame = Frame(gui_scene)

    if launch_gepetto_gui_exec:
        # close the window after little while
        time.sleep(5)
        GepettoGuiScene.close_gepetto_gui(gepetto_gui_process)

    return gui_scene, bolt_visual, world_frame


if __name__ == "__main__":
    display_bolt_in_gepetto_gui()
