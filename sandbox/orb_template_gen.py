#!/usr/bin/env python
import ecto
from ecto_opencv import cv_bp
from ecto_opencv.highgui import imread, MatWriter
from ecto_opencv.features2d import ORB, ORBstats, DescriptorAccumulator, KeypointsToMat
from ecto_opencv.imgproc import cvtColor, Conversion
from ecto_opencv.calib import PointsTo3d
from ecto.opts import scheduler_options, run_plasm, cell_options
import os
import numpy as np
import matplotlib.pyplot as plt
import shutil

def parse_args():
    import argparse
    parser = argparse.ArgumentParser(description='Test orb on images.')
    parser.add_argument('-i,--input', dest='input',
                        help='The input dir. %(default)s', default='./images')
    parser.add_argument('-o,--output', dest='output', type=str,
                        help='The output directory for this template. Default: %(default)s', default='./')
    factory = cell_options(parser, ORB, 'ORB')
    scheduler_options(parser.add_argument_group('Scheduler'), default_niter=1)
    options = parser.parse_args()
    options.niter = 1
    options.orb_factory = factory
    return options

options = parse_args()

image = imread(image_file=options.input)
shutil.copy(options.input, os.path.join(options.output, 'train.png'))
orb_m = options.orb_factory(options)
rgb2gray = cvtColor (flag=Conversion.RGB2GRAY)
kpts2mat = KeypointsToMat()
ptsTo3d = PointsTo3d(scale=0.0254 / 100) #100 dpi ~> 25.4 mm/ 100 px
plasm = ecto.Plasm()
plasm.connect(image['image'] >> orb_m['image'],
              orb_m['keypoints'] >> kpts2mat['keypoints'],
              kpts2mat['points'] >> ptsTo3d['points']
              )

if not os.path.exists(options.output):
    print 'making ', options.output
    os.makedirs(options.output)

#training 
points3d_writer = MatWriter(filename=os.path.join(options.output, 'points3d.yaml'))
points_writer = MatWriter(filename=os.path.join(options.output, 'points.yaml'))
descriptor_writer = MatWriter(filename=os.path.join(options.output, 'descriptors.yaml'))
R_writer = MatWriter(filename=os.path.join(options.output, 'R.yaml'))
T_writer = MatWriter(filename=os.path.join(options.output, 'T.yaml'))
m = cv_bp.Mat()
m.fromarray(np.eye(3, 3, dtype=np.float64))
R_writer.inputs.mat = m
m = cv_bp.Mat()
m.fromarray(np.zeros((3, 1), dtype=np.float64))
T_writer.inputs.mat = m

for y, x in (
            (orb_m['descriptors'], descriptor_writer),
            (kpts2mat['points'], points_writer),
            (ptsTo3d['points3d'], points3d_writer)
            ):
    plasm.connect(y >> x['mat'],
              )
T_writer.process()
R_writer.process()
run_plasm(options, plasm, locals=vars())
