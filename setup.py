#!/usr/bin/env python
from distutils.core import setup

setup(name='object_recognition_capture',
      version='1.0.0',
      description='A simple set of scripts to capture a 3d model of an object',
      packages=['object_recognition_capture', 'object_recognition_capture.arbotix'],
      package_dir={'':'python'}
)
