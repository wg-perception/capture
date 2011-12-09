#!/usr/bin/env python
import ecto
from ecto_opencv import cv_bp
from ecto_opencv.imgproc import Scale, AREA
from ecto_opencv.highgui import imread, ImageMode, imshow, ImageSaver
from ecto_opencv.features2d import ORB, Matcher, KeypointsToMat, MatchRefinement, DrawMatches, MatchesToMat
from ecto.opts import scheduler_options, run_plasm
import numpy as np
import os
def parse_args():
    import argparse
    parser = argparse.ArgumentParser(description='Test orb on images.')
    parser.add_argument('train', help='train image.')
    parser.add_argument('test', help='test image.')
    parser.add_argument('output', help='the output dir.')
    scheduler_options(parser.add_argument_group('Scheduler'), default_niter=1)
    options = parser.parse_args()
    return options

def hookup_orb(plasm, source, orb):
    kpts = KeypointsToMat()
    plasm.connect(source['image'] >> orb['image'],
                  orb['keypoints'] >> kpts['keypoints']
                  )
    return kpts

class MatchEval(ecto.Cell):
    def declare_params(self, p):
        p.declare('output', 'Output dir', 'output')

    def declare_io(self, p, i, o):
        i.declare('matches', 'Matches mat')
        i.declare('train', 'Train points')
        i.declare('test', 'Test points')
        i.declare('inliers', 'inliers mask for the matches')

    def process(self, i, o):
        matches = i.matches.toarray()
        train = i.train.toarray()
        test = i.test.toarray()
        inliers = i.inliers.toarray()
        o = self.params.output
        join = os.path.join
        np.savetxt(join(o, 'matches.txt'), matches)
        np.savetxt(join(o, 'train.txt'), train)
        np.savetxt(join(o, 'test.txt'), test)
        np.savetxt(join(o, 'inliers.txt'), inliers)
        return ecto.OK
def drange(start, stop, step):
    r = start
    while r < stop:
        yield r
        r += step

def match(options,
        scale=1.0,
        train_scale=1 / 3.0,
        scale_factor=1.05,
        n_levels=5,
        n_features_train=10000,
        n_features_test=1000,
        match_distance=120,
    ):  

    if not os.path.exists(options.output):
        os.makedirs(options.output)

    train = imread('Train image', image_file=options.train, mode=ImageMode.GRAYSCALE)
    test = imread('Test image', image_file=options.test, mode=ImageMode.GRAYSCALE)
    train_name, _extension = os.path.splitext(os.path.basename(options.train))
    test_name, _extension = os.path.splitext(os.path.basename(options.test))

    print 'Scale %d is %f' % (0, scale)
    for i in range(n_levels - 1):
        scale *= 1.0 / scale_factor
        print 'Scale %d is %f' % (i + 1, scale)
    orb_train = ORB(n_features=n_features_train, n_levels=n_levels, scale_factor=scale_factor)
    orb_test = ORB(n_features=n_features_test, n_levels=1)
    
    
    
    plasm = ecto.Plasm()
    scale = Scale(factor=train_scale, interpolation=AREA)
    plasm.connect(train['image'] >> scale['image'])
    
    train_pts = hookup_orb(plasm, scale, orb_train)
    test_pts = hookup_orb(plasm, test, orb_test)
    
    matcher = Matcher()
    plasm.connect(orb_train['descriptors'] >> matcher['train'],
                 orb_test['descriptors'] >> matcher['test'],
                 )
    
    matchesMat = MatchesToMat()
    h_est = MatchRefinement(match_distance=match_distance)
    plasm.connect(train_pts['points'] >> h_est['train'],
                  test_pts['points'] >> h_est['test'],
                  matcher['matches'] >> h_est['matches'],
                  h_est['matches'] >> matchesMat['matches']
                  )
    
    match_draw = DrawMatches()
    output_base = os.path.join(options.output, '%(train)s_%(test)s' % dict(train=train_name, test=test_name))
    match_image = output_base + '.png'
    image_saver = ImageSaver(filename=match_image)
    
    plasm.connect(train_pts['points'] >> match_draw['train'],
                  test_pts['points'] >> match_draw['test'],
                  h_est['matches'] >> match_draw['matches'],
                  h_est['matches_mask'] >> match_draw['matches_mask'],
                  scale['image'] >> match_draw['train_image'],
                  test['image'] >> match_draw['test_image'],
                  match_draw[:] >> image_saver[:]
                  )


    match_eval = MatchEval(output=options.output)
    plasm.connect(matchesMat['matches'] >> match_eval['matches'],
                  train_pts['points'] >> match_eval['train'],
                  test_pts['points'] >> match_eval['test'],
                  h_est['matches_mask'] >> match_eval['inliers'],
                  )
    run_plasm(options, plasm, locals=vars())


options = parse_args()
output = options.output
for train_scale in drange(0.1, 1, 0.02):
    options.output = '%s_%f' % (output, train_scale)
    match(options, train_scale=train_scale)
