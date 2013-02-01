from orb_capture import OrbPoseEstimator
from ecto_opencv import highgui, calib, imgproc
from ecto_ros import Cv2CameraInfo, Mat2Image, RT2PoseStamped
from fiducial_pose_est import OpposingDotPoseEstimator
from ecto_image_pipeline.base import CameraModelToCv
from ecto_image_pipeline.io.source import create_source
from ecto_opencv.calib import DepthTo3d
import object_recognition_capture
import ecto
import ecto_ros
from ecto_ros.ecto_sensor_msgs import Bagger_Image as ImageBagger, Bagger_CameraInfo as CameraInfoBagger
from ecto_ros.ecto_geometry_msgs import Bagger_PoseStamped as PoseBagger
import math
import time
from ecto_openni import SXGA_RES, VGA_RES, FPS_30, FPS_15

class TurnTable(ecto.Cell):
    '''Uses the arbotix library to talk to servoes.'''

    @staticmethod
    def declare_params(params):
        params.declare('angle_thresh', 'Angular threshold', math.pi / 36)

    @staticmethod
    def declare_io(_p, _i, o):
        o.declare('trigger', 'Capture', True)

    def configure(self, params):
        try:
            self.MY_SERVO = 0xE9
            self.a = ArbotiX('/dev/ttyUSB0', baud=1e6) #1 meg for e
            self.a.disableTorque(self.MY_SERVO)
            self.a.disableWheelMode(self.MY_SERVO, resolution=12)
            pos = self.a.getPosition(self.MY_SERVO)
            self.a.setSpeed(self.MY_SERVO, 100)
            self.a.setPosition(self.MY_SERVO, 0)
            while self.a.getPosition(self.MY_SERVO) > 5:
                time.sleep(0.1)
            print 'At position: ', self.a.setPosition(self.MY_SERVO, 0)
            self.settle_count = -1 #use to alternate movement
            self.total = 0
            #print params
            self.delta_angle = params.angle_thresh * 180 / math.pi
            self.max_angle = 720
            print self.delta_angle, 'delta angle.'
        except Exception, e:
            print e

    def process(self, _i, outputs):
        if self.settle_count == 0:
            self.settle_count += 1
            self.total += 1
            rotate(self.a, self.MY_SERVO, self.delta_angle, 25)
            print 'Total angular travel :', self.total * self.delta_angle
            outputs.trigger = False
        elif self.settle_count >= 2:
            outputs.trigger = True
            self.settle_count = 0
        else:
            self.settle_count += 1
            time.sleep(0.25)
        if self.total * self.delta_angle > self.max_angle:
            return 1
        return 0

    def __del__(self):
        print 'Stopping servo....'
        a = ArbotiX('/dev/ttyUSB0', baud=1e6)
        MY_SERVO = 0xE9
        a.disableTorque(MY_SERVO)

def create_capture_plasm(bag_name, angle_thresh, segmentation_cell, n_desired=72,
                                            orb_template='', res=SXGA_RES, fps=FPS_30,
                                            orb_matches=False,
                                            preview=False, use_turn_table=True):
    '''
    Creates a plasm that will capture openni data into a bag, using a dot pattern to sparsify views.
    
    @param bag_name: A filename for the bag, will write to this file.
    @param angle_thresh: The angle threshhold in radians to sparsify the views with.  
    '''
    graph = []

    # try several parameter combinations
    source = create_source('image_pipeline','OpenNISource',image_mode=res,image_fps=fps)

    poser = OpposingDotPoseEstimator(rows=5, cols=3,
                                     pattern_type=calib.ASYMMETRIC_CIRCLES_GRID,
                                     square_size=0.04, debug=True)
    depth_to_3d = DepthTo3d()

    if orb_template != '':
        poser = OrbPoseEstimator(directory=orb_template, show_matches=orb_matches)
        graph += [source['depth_raw'] >> depth_to_3d['depth'],
                  source['K'] >> depth_to_3d['K'],
                  depth_to_3d['points3d'] >> poser['points3d'],
                  source['mask_depth'] >> poser['mask'],
                  ]
    rgb2gray = imgproc.cvtColor('rgb -> gray', flag=imgproc.Conversion.RGB2GRAY)
    delta_pose = ecto.If('delta R|T', cell=object_recognition_capture.DeltaRT(angle_thresh=angle_thresh,
                                                          n_desired=n_desired))

    display = highgui.imshow(name='Poses')
    poseMsg = RT2PoseStamped(frame_id='/camera_rgb_optical_frame')

    graph += [source['image'] >> rgb2gray[:],
              source['image'] >> poser['color_image'],
              rgb2gray[:] >> poser['image'],
              source['K'] >> poser['K'],
              poser['debug_image'] >> (display['image'],),
              poser['R', 'T', 'found'] >> delta_pose['R', 'T', 'found'],
              poser['R', 'T'] >> poseMsg['R', 'T'],
              ]

    masker = segmentation_cell
    maskMsg = Mat2Image(frame_id='/camera_rgb_optical_frame')
    rgbMsg = Mat2Image(frame_id='/camera_rgb_optical_frame', swap_rgb=True)
    depthMsg = Mat2Image(frame_id='/camera_rgb_optical_frame')
    
    
    graph += [
              source['depth'] >> depthMsg[:],
              source['image'] >> rgbMsg[:],
              ]
    graph += [
              source['depth'] >> masker['depth'],
              source['K'] >> masker['K'],
              poser['R', 'T'] >> masker['R', 'T'],
              masker['mask'] >> maskMsg[:],
              ]
#    graph += [source['depth'] >> highgui.imshow(name='depth')['image']
#              ]
    camera2cv = CameraModelToCv()
    cameraMsg = Cv2CameraInfo(frame_id='/camera_rgb_optical_frame')
    graph += [source['camera'] >> camera2cv['camera'],
              camera2cv['K', 'D', 'image_size'] >> cameraMsg['K', 'D', 'image_size']
              ]
    #display the mask
    mask_and = imgproc.BitwiseAnd()
    mask2rgb = imgproc.cvtColor('mask -> rgb', flag=imgproc.Conversion.GRAY2RGB)
    mask_display = highgui.imshow(name='mask')
    graph += [
              masker['mask'] >> mask2rgb['image'],
              mask2rgb['image'] >> mask_and['a'],
              source['image'] >> mask_and['b'],
              mask_and[:] >> mask_display['image'],
            ]
    if not preview:
        baggers = dict(image=ImageBagger(topic_name='/camera/rgb/image_color'),
                   depth=ImageBagger(topic_name='/camera/depth/image'),
                   mask=ImageBagger(topic_name='/camera/mask'),
                   pose=PoseBagger(topic_name='/camera/pose'),
                   image_ci=CameraInfoBagger(topic_name='/camera/rgb/camera_info'),
                   depth_ci=CameraInfoBagger(topic_name='/camera/depth/camera_info'),
                   )
        bagwriter = ecto.If('Bag Writer if R|T',
                            cell=ecto_ros.BagWriter(baggers=baggers, bag=bag_name)
                            )

        graph += [
                  rgbMsg[:] >> bagwriter['image'],
                  depthMsg[:] >> bagwriter['depth'],
                  cameraMsg[:] >> (bagwriter['image_ci'], bagwriter['depth_ci']),
                  poseMsg['pose'] >> bagwriter['pose'],
                  maskMsg[:] >> bagwriter['mask'],
                  ]
        novel = delta_pose['novel']
        if use_turn_table:
            table = TurnTable(angle_thresh=angle_thresh)
            ander = ecto.And()
            graph += [
                  table['trigger'] >> (delta_pose['__test__'], ander['in2']),
                  delta_pose['novel'] >> ander['in1'],
                  ]
            novel = ander['out']
        else:
            delta_pose.inputs.__test__ = True

        graph += [novel >> (bagwriter['__test__'])]

    plasm = ecto.Plasm()
    plasm.connect(graph)
    return (plasm, segmentation_cell) # return segmentation for tuning of parameters.
