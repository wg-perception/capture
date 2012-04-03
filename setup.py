#!/usr/bin/env python
from distutils.core import setup

setup(name='Capture',
      version='1.0.0',
      description='A simple set of scripts to capture a 3d model of an object',
      packages=['capture', 'cpture.arbotix'],
      package_dir={'':'python'}
)
