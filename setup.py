from distutils.core import setup
from catkin_pkg.python_setup import generate_distutils_setup

setup_args = generate_distutils_setup(
    packages=['ontological_robot_control', 'armor_api'],
    package_dir={'': 'utilities'}
)

setup(**setup_args)
