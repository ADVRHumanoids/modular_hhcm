#!/usr/bin/env python

# from distutils.core import setup
from setuptools import setup

from catkin_pkg.python_setup import generate_distutils_setup

d = generate_distutils_setup(
   # This will make the RobotDesignStudio.py script globally visible
   # Not needed if using $ env FLASK_APP=modular flask run
   # scripts=['src/modular/RobotDesignStudio.py'],
   packages=['modular', 'modular.optimization', 'modular.protobuf', 'modular.web'],
   package_dir={'': 'src'},
   # package_data={'modular': ['*.yaml', 'templates/*', 'static/*']},  # Alternative to include_package_data
   include_package_data=True,
   # data_files=[('launch', ['launch/*'])],  # Add files outside package. Not working
   install_requires=['setuptools', 'flask', 'anytree'],
   python_requires='==2.7.*'
)

setup(**d)