"""Installation script for the 'v1' python package."""

import os
import toml

from setuptools import setup


# Installation operation
setup(
    name="v1",
    packages=["v1"],
    license="MIT",
    include_package_data=True,
    python_requires=">=3.10",
    classifiers=[
        "Natural Language :: English",
        "Programming Language :: Python :: 3.10",
        "Isaac Sim :: 2023.1.1",
        "Isaac Sim :: 4.0.0",
    ],
    zip_safe=False,
)
