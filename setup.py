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
   zip_safe=False,
   # data_files=[('launch', ['launch/*'])],  # Add files outside package. Not working
   install_requires=['protobuf_to_dict>=0.1.0', 'protobuf==3.17.3', 'flask', 'anytree', 'pyzmq', 'future', 'rospkg',  'defusedxml'],   # , 'transformations'], todo: import guard for transformations or tf.transformations  
   python_requires='==2.7.*'
)

setup(**d)
