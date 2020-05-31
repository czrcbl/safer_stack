#!/usr/bin/env python
from distutils.core import setup
from catkin_pkg.python_setup import generate_distutils_setup

setup_args = generate_distutils_setup(
     packages=['safer_stack'],
     package_dir={'': 'src'},
     install_requires=[
          'numpy',
          'mxnet-cu102',
          'gluoncv',
          'opnecv-contrib-python'
      ]
)

setup(**setup_args)