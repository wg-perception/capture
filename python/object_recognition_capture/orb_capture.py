"""
Module defining a few BlackBoxes for textured plane tracking
"""
from ecto import BlackBoxCellInfo, BlackBoxForward
from ecto_opencv.calib import Select3d, Select3dRegion, PoseDrawer, TransformCompose
from ecto_opencv.features2d import FASTFeature, ORB, Matcher, MatchRefinementHSvd, DrawMatches, KeypointsToMat, \
    LSHMatcher
from ecto_opencv.highgui import imshow, FPSDrawer, MatReader, imread
import ecto

class FeatureFinder(ecto.BlackBox):
    @classmethod
    def declare_cells(cls, _p):
        return {'orb': BlackBoxCellInfo(ORB),
                'fast': BlackBoxCellInfo(FASTFeature),
                'image': BlackBoxCellInfo(ecto.Passthrough),
                'keypointsTo2d': BlackBoxCellInfo(KeypointsToMat),
                'select3d': BlackBoxCellInfo(Select3d)}

    @classmethod
    def declare_direct_params(cls, p, **_kwargs):
        p.declare('use_fast', 'Use fast or not.', False)

    @classmethod
    def declare_forwards(cls, p_in):
        p = {'orb': 'all', 'fast': 'all'}
        i = {}
        if p_in.use_fast:
            i['fast'] = [BlackBoxForward('mask')]
            i['image'] = [BlackBoxForward('in','image')]
        else:
            i['orb'] = 'all'
        i['select3d'] = [BlackBoxForward('points3d')]
        o = {'orb': 'all', 'keypointsTo2d' : 'all', 'select3d' : 'all'}
        return (p, i, o)

    def connections(self, p):
        graph = [self.orb['keypoints'] >> self.keypointsTo2d['keypoints'],
                self.keypointsTo2d['points'] >> self.select3d['points'], ]
        if p.use_fast:
            graph += [self.image[:] >> (self.fast['image'], self.orb['image']),
                      self.fast['keypoints'] >> self.orb['keypoints'],
                      ]
        return graph

class TemplateLoader(ecto.BlackBox):
    @classmethod
    def declare_cells(cls, p):
        return {'points': BlackBoxCellInfo(MatReader, {'filename': '%s/points.yaml' % p.directory}),
                'points3d': BlackBoxCellInfo(MatReader, {'filename': '%s/points3d.yaml' % p.directory}),
                'descriptors': BlackBoxCellInfo(MatReader, {'filename': '%s/descriptors.yaml' % p.directory}),
                'R': BlackBoxCellInfo(MatReader, {'filename': '%s/R.yaml' % p.directory}),
                'T': BlackBoxCellInfo(MatReader, {'filename': '%s/T.yaml' % p.directory}),
                'image': BlackBoxCellInfo(imread, {'image_file': '%s/train.png' % p.directory})}

    @classmethod
    def declare_direct_params(cls, p):
        p.declare('directory', 'The directory of the template.', '.')

    @staticmethod
    def declare_forwards(_p):
        o = {}
        for x in ('points', 'points3d', 'descriptors', 'R', 'T'):
            o[x] = [BlackBoxForward('mat', x)]
        o['image'] = [BlackBoxForward('image')]

        return ({},{},o)

    def connections(self, _p):
        return [self.points, self.points3d, self.descriptors, self.R, self.T, self.image]

class OrbPoseEstimator(ecto.BlackBox):
    '''Estimates the pose of an ORB based template.
    '''
    @classmethod
    def declare_cells(cls, _p):
        return {'depth_mask': BlackBoxCellInfo(ecto.Passthrough),
                'fps': BlackBoxCellInfo(FPSDrawer),
                'gray_image': BlackBoxCellInfo(ecto.Passthrough),
                'K': BlackBoxCellInfo(ecto.Passthrough),
                'lsh': BlackBoxCellInfo(LSHMatcher),
                'orb': BlackBoxCellInfo(FeatureFinder),
                'pose_estimation': BlackBoxCellInfo(MatchRefinementHSvd),
                'points3d': BlackBoxCellInfo(ecto.Passthrough),
                'rgb_image': BlackBoxCellInfo(ecto.Passthrough),
                'tr': BlackBoxCellInfo(TransformCompose)}

    @classmethod
    def declare_direct_params(cls, p):
        p.declare('directory', 'The template directory.', '.')
        p.declare('show_matches', 'Display the matches.', False)
        p.declare('use_lsh', 'Use lsh for matching instead of brute force.', True)

    @staticmethod
    def declare_forwards(_p):
        #inputs
        p = {'lsh': 'all', 'orb': 'all', 'pose_estimation': 'all'}
        i = {}
        for cell_name_new_key in [('K','K'), ('rgb_image','color_image'),
                                  ('gray_image', 'image'), ('depth_mask', 'mask'),
                                  ('points3d', 'points3d')]:
            cell_name, new_key = cell_name_new_key
            i[cell_name] = [BlackBoxForward('in', new_key)]

        #outputs
        o = {'tr': [BlackBoxForward('R'), BlackBoxForward('T')],
             'pose_estimation': [BlackBoxForward('found')],
             'fps': [BlackBoxForward('image', 'debug_image')]}

        return (p, i, o)

    def configure(self, p, _i, _o):
        self.train = TemplateLoader(directory=p.directory)
        self.show_matches = p.show_matches
        self.use_lsh = p.use_lsh

    def connections(self, _p):
        train = self.train
        orb = self.orb
        graph = [ self.gray_image[:] >> orb['image'],
                  self.points3d[:] >> orb['points3d'],
                  self.depth_mask[:] >> orb['mask']
                ]

        matcher = Matcher()
#        if self.use_lsh:
#           matcher = self.lsh
        graph += [ orb['descriptors'] >> matcher['test'],
                   train['descriptors'] >> matcher['train'],
                  ]

        #3d estimation
        pose_estimation = self.pose_estimation
        graph += [matcher['matches'] >> pose_estimation['matches'],
                  orb['points'] >> pose_estimation['test_2d'],
                  train['points'] >> pose_estimation['train_2d'],
                  orb['points3d'] >> pose_estimation['test_3d'],
                  train['points3d'] >> pose_estimation['train_3d'],
                  ]

        if self.show_matches:
            #display matches
            match_drawer = DrawMatches()
            graph += [pose_estimation['matches'] >> match_drawer['matches'],
                      pose_estimation['matches_mask'] >> match_drawer['matches_mask'],
                      orb['points'] >> match_drawer['test'],
                      train['points'] >> match_drawer['train'],
                      self.rgb_image[:] >> match_drawer['test_image'],
                      train['image'] >> match_drawer['train_image'],
                      match_drawer['output'] >> imshow(name='Matches')['']
                      ]

        tr = self.tr
        fps = self.fps
        pose_draw = PoseDrawer()
        graph += [train['R', 'T'] >> tr['R1', 'T1'],
                  pose_estimation['R', 'T'] >> tr['R2', 'T2'],
                  tr['R', 'T'] >> pose_draw['R', 'T'],
                  pose_estimation['found'] >> pose_draw['trigger'],
                  self.K[:] >> pose_draw['K'],
                  self.rgb_image[:] >> pose_draw['image'],
                  pose_draw['output'] >> fps[:],
                  ]
        return graph
