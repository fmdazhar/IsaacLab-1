# Copyright (c) 2022-2024, The Isaac Lab Project Developers.
# All rights reserved.
#
# SPDX-License-Identifier: BSD-3-Clause

"""Installation script for the 'omni.isaac.lab_tasks' python package."""

import itertools
import os
import platform
import toml

from setuptools import setup

# Obtain the extension data from the extension.toml file
EXTENSION_PATH = os.path.dirname(os.path.realpath(__file__))
# Read the extension.toml file
EXTENSION_TOML_DATA = toml.load(os.path.join(EXTENSION_PATH, "config", "extension.toml"))

# Minimum dependencies required prior to installation
INSTALL_REQUIRES = [
    # generic
    "numpy",
    "torch==2.4.0",
    "torchvision>=0.14.1",  # ensure compatibility with torch 1.13.1
    # 5.26.0 introduced a breaking change, so we restricted it for now.
    # See issue https://github.com/tensorflow/tensorboard/issues/6808 for details.
    "protobuf>=3.20.2, < 5.0.0",
    # configuration management
    "hydra-core",
    # data collection
    "h5py",
    # basic logger
    "tensorboard",
    # video recording
    "moviepy",
]

PYTORCH_INDEX_URL = ["https://download.pytorch.org/whl/cu118"]

# Installation operation
setup(
    name="omni-isaac-lab_tasks",
    author="Isaac Lab Project Developers",
    maintainer="Isaac Lab Project Developers",
    url=EXTENSION_TOML_DATA["package"]["repository"],
    version=EXTENSION_TOML_DATA["package"]["version"],
    description=EXTENSION_TOML_DATA["package"]["description"],
    keywords=EXTENSION_TOML_DATA["package"]["keywords"],
    include_package_data=True,
    python_requires=">=3.10",
    install_requires=INSTALL_REQUIRES,
    dependency_links=PYTORCH_INDEX_URL,
    packages=["omni.isaac.lab_tasks"],
    classifiers=[
        "Natural Language :: English",
        "Programming Language :: Python :: 3.10",
        "Isaac Sim :: 4.2.0",
        "Isaac Sim :: 4.1.0",
    ],
    zip_safe=False,
)
