from setuptools import find_packages
from setuptools import setup

setup(
    name='parc_robot_bringup',
    version='0.0.0',
    packages=find_packages(
        include=('parc_robot_bringup', 'parc_robot_bringup.*')),
)
