import ecto
from ecto_opencv import highgui, calib, imgproc
from ecto import BlackBoxForward as Forward

class OpposingDotPoseEstimator(ecto.BlackBox):
    '''Estimates the pose of a fiducial that has a black on white pattern and a white on black pattern.
    TODO support other configurations...
    '''
    @staticmethod
    def declare_cells(p):
        #TODO parameterize the quantizer
        #abuse saturated Arithmetics http://opencv.itseez.com/modules/core/doc/intro.html?highlight=saturated.
        cells = {'gray_image': ecto.Passthrough('gray Input'),
                'rgb_image': ecto.Passthrough('rgb Input'),
                'camera_info': ecto.Passthrough('K_image'),
                'gather': calib.GatherPoints("gather", N=2),
                'quantizer': imgproc.Quantize('Quantizer', alpha=1, beta=0),
                'invert': imgproc.BitwiseNot()}

        offset_x = -.3095 #TODO: FIXME hard coded
        offset_y = -.1005
        cells['cd_bw'] = calib.PatternDetector('Dot Detector, B/W',
                                                rows=p.rows, cols=p.cols,
                                                pattern_type=p.pattern_type,
                                                square_size=p.square_size,
                                                offset_x=offset_x,
                                                offset_y=offset_y,
                                                )
        offset_x = .1505 #TODO: FIXME hard coded
        cells['cd_wb'] = calib.PatternDetector('Dot Detector, W/B',
                                                rows=p.rows, cols=p.cols,
                                                pattern_type=p.pattern_type,
                                                square_size=p.square_size,
                                                offset_x=offset_x,
                                                offset_y=offset_y,
                                                )
        cells['pose_calc'] = calib.FiducialPoseFinder('Pose Calc')
        cells['circle_drawer'] = calib.PatternDrawer('Circle Draw',
                                                 rows=p.rows, cols=p.cols)
        cells['circle_drawer2'] = calib.PatternDrawer('Circle Draw',
                                                 rows=p.rows, cols=p.cols)
        cells['fps'] = highgui.FPSDrawer()

        return cells
 
    @staticmethod
    def declare_direct_params(p):
        p.declare('rows', 'Number of rows in the pattern.', 11)
        p.declare('cols', 'Number of cols in the pattern.', 7)
        p.declare('pattern_type', 'Type of pattern', calib.ASYMMETRIC_CIRCLES_GRID)
        p.declare('square_size', 'The pattern spacing', 0.1)
        p.declare('debug', 'Debug the detector.', True)

    @staticmethod
    def declare_forwards(_p):
        #inputs
        i = {'gray_image': [Forward('in','image')],
             'rgb_image': [Forward('in','color_image')],
             'camera_info': [Forward('in','K_image')]}

        #outputs
        o = {'pose_calc': [Forward('R'), Forward('T')],
             'gather': [Forward('found')],
             'fps': [Forward('image', 'debug_image')]}

        return ({},i,o)

    def connections(self, p):
        graph = [
                self.gray_image[:] >> self.quantizer[:],
                self.quantizer[:] >> (self.invert[:], self.cd_bw['input']),
                self.cd_bw['found', 'ideal', 'out'] >> self.gather['found_0000', 'ideal_0000', 'points_0000'],
                self.cd_wb['found', 'ideal', 'out'] >> self.gather['found_0001', 'ideal_0001', 'points_0001'],
                self.invert[:] >> self.cd_wb['input'],
                self.gather['out', 'ideal', 'found'] >> self.pose_calc['points', 'ideal', 'found'],
                self.camera_info[:] >> self.pose_calc['K'],
                ]
        if p.debug:
            graph += [self.rgb_image[:] >> self.circle_drawer['input'],
                      self.circle_drawer[:] >> self.circle_drawer2['input'],
                      self.cd_bw['out', 'found'] >> self.circle_drawer['points', 'found'],
                      self.cd_wb['out', 'found'] >> self.circle_drawer2['points', 'found'],
                      self.circle_drawer2[:] >> self.fps[:]
#                      self.quantizer[:] >> highgui.imshow(name='quantized')[:],
             ]
        return graph
