#!/usr/bin/env python

from distutils.core import setup
from catkin_pkg.python_setup import generate_distutils_setup

setup_args = generate_distutils_setup(
    packages=["kinematics"], package_dir={"": "src"}, tests_require=["unittest"]
)

setup(**setup_args)
