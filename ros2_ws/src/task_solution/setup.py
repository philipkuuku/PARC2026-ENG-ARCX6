from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'task_solution'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # Launch files
        (
            os.path.join('share', package_name, 'launch'),
            glob('launch/*.py')
        ),
        # Config files
        (
            os.path.join('share', package_name, 'config'),
            glob('config/*')
        ),
        # Map files
        (
            os.path.join('share', package_name, 'maps'),
            glob('maps/*')
        ),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='ARCX6',
    maintainer_email='eyiramgaze@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'green_detector = task_solution.green_detector:main',
            'green_nav = task_solution.green_nav:main',
            'green_nav2 = task_solution.green_nav2:main',
        ],
    },
)
