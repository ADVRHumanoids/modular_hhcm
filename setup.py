#!/usr/bin/env python

# from distutils.core import setup
from setuptools import setup

from catkin_pkg.python_setup import generate_distutils_setup

d = generate_distutils_setup(
   ##  don't do this unless you want a globally visible script
   # scripts=['bin/myscript'], 
   packages=['modular', 'modular.optimization', 'modular.protobuf'],
   package_dir={'': 'src'},
   # package_data={'modular': ['*.yaml', 'templates/*', 'static/*']},  # Alternative to include_package_data
   include_package_data=True,
   install_requires=['setuptools', 'flask', 'anytree'],
   python_requires='==2.7.*'
)

setup(**d)