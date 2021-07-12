#!/usr/bin/env python
"""setup.py
License BSD-3-Clause
Copyright (c) 2021, New York University and Max Planck Gesellschaft.
"""

import sys
from os import path, walk, getcwd
from shutil import copytree, rmtree
from pathlib import Path
from distutils.core import setup
from distutils.command.build_py import build_py
from setuptools import setup, find_packages
from setuptools.command.install import install
from setuptools.command.develop import develop
from setuptools.command.egg_info import egg_info


# Defines the paramters of this package:
package_name = "robot_properties_bolt"
package_version = "1.0.0"


def print_error(*args, **kwargs):
    """ Print in stderr. """
    print(*args, file=sys.stderr, **kwargs)


def find_resources(package_name):
    """ Find the relative path of files under the resource folder. """
    resources = []
    package_dir = path.join("src", package_name)
    resources_dir = path.join(package_dir, package_name)

    for (root, _, files) in walk(resources_dir):
        for afile in files:
            if (afile != package_name and 
                not afile.endswith(".DS_Store") and
                not afile.endswith(".py")):
                rel_dir = path.relpath(root, package_dir)
                src = path.join(rel_dir, afile)
                resources.append(src)
    return resources


# Long description from the readme.
with open(path.join(path.dirname(path.realpath(__file__)), "readme.md"), "r") as fh:
    long_description = fh.read()

# Find the resource files.
resources = find_resources(package_name)

# Install the package.xml.
data_files_to_install = [(path.join("share", package_name), ["package.xml"])]
data_files_to_install += [
    ("share/ament_index/resource_index/packages", 
    [path.join("src", package_name, package_name, package_name)])
]

# Install nodes and demos.
scripts_list = []
for (root, _, files) in walk(path.join("demos")):
    for demo_file in files:
        scripts_list.append(path.join(root, demo_file))

class custom_build_py(build_py):
    def run(self):

        # Try to build the doc and install it.
        try:
            # Get the mpi_cmake_module build doc method
            from mpi_cmake_modules.documentation_builder import (
                build_documentation,
            )

            build_documentation(
                str(
                    (
                        Path(self.build_lib) / package_name / "doc"
                    ).absolute()
                ),
                str(Path(__file__).parent.absolute()),
                package_version,
            )
        except ImportError as e:
            print_error()

        # distutils uses old-style classes, so no super()
        build_py.run(self)

# Final setup.
setup(
    name=package_name,
    version=package_version,
    package_dir={package_name: path.join("src", package_name)},
    packages=[package_name],
    package_data={package_name: resources},
    data_files=data_files_to_install,
    scripts=scripts_list,
    install_requires=["setuptools", 
                      "xacro", 
                      "pybullet", 
                      "importlib_resources",
                      "meshcat"],
    zip_safe=True,
    maintainer="mnaveau",
    maintainer_email="mnaveau@tuebingen.mpg.de",
    long_description=long_description,
    long_description_content_type="text/markdown",
    url="https://github.com/open-dynamic-robot-initiative/robot_properties_bolt",
    description="Wrapper around the pybullet interface using pinocchio.",
    license="BSD-3-clause",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [],
    },
    classifiers=[
        "Programming Language :: Python :: 3",
        "License :: OSI Approved :: BSD-3-clause",
        "Operating System :: OS Independent",
    ],
    python_requires=">=3.6",
    cmdclass={"build_py": custom_build_py},
)
