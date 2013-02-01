#!/usr/bin/env python
import ecto

from ecto_opencv.highgui import imshow
from ecto_opencv.imgproc import cvtColor, Conversion
from ecto_opencv.calib import DepthTo3d
from ecto.opts import scheduler_options, run_plasm, cell_options
from ecto_image_pipeline.io.source import create_source

from object_recognition_capture.orb_capture import OrbPoseEstimator

if __name__ == '__main__':
    def parse_args():
        import argparse
        parser = argparse.ArgumentParser(description='Estimate the pose of an ORB template.')

        scheduler_options(parser.add_argument_group('Scheduler'))
        factory = cell_options(parser, OrbPoseEstimator, 'track')
        options = parser.parse_args()
        options.orb_factory = factory
        return options

    options = parse_args()
    plasm = ecto.Plasm()

    #setup the input source, grayscale conversion
    from ecto_openni import SXGA_RES, FPS_15
    source = create_source('image_pipeline','OpenNISource',image_mode=SXGA_RES,image_fps=FPS_15)
    rgb2gray = cvtColor('Grayscale', flag=Conversion.RGB2GRAY)
    depth_to_3d = DepthTo3d()
    plasm.connect(source['image'] >> rgb2gray ['image'])

    pose_est = options.orb_factory(options, 'ORB Tracker')

    #convenience variable for the grayscale
    img_src = rgb2gray['image']

    #connect up the pose_est
    plasm.connect(img_src >> pose_est['image'],
                  source['image'] >> pose_est['color_image'],
                  source['depth_raw'] >> depth_to_3d['depth'],
                  source['K'] >> depth_to_3d['K'],
                  depth_to_3d['points3d'] >> pose_est['points3d'],
                  source['mask_depth'] >> pose_est['mask'],
                  source['K'] >> pose_est['K']
                  )

    display = imshow('orb display', name='Pose')
    plasm.connect(pose_est['debug_image'] >> display['image'],
                  )

    run_plasm(options, plasm, locals=vars())
