"""
Module defining common tools for object capture
"""
from ecto_image_pipeline.base import CameraModelToCv
from ecto_image_pipeline.io.source import create_source
from ecto_opencv import highgui, calib, imgproc
from ecto_opencv.calib import DepthTo3d
from ecto_opencv.rgbd import ComputeNormals, PlaneFinder
from ecto_openni import SXGA_RES, FPS_30
from ecto_ros import Cv2CameraInfo, Mat2Image, RT2PoseStamped
from ecto_ros.ecto_geometry_msgs import Bagger_PoseStamped as PoseBagger
from ecto_ros.ecto_sensor_msgs import Bagger_Image as ImageBagger, Bagger_CameraInfo as CameraInfoBagger
from fiducial_pose_est import OpposingDotPoseEstimator
from orb_capture import OrbPoseEstimator
import ecto
import ecto_ros
import math
import object_recognition_capture
import time

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
    source = create_source('image_pipeline', 'OpenNISource', outputs_list=['K', 'camera', 'image', 'depth', 'points3d',
                                                                           'mask_depth'], res=res, fps=fps)

    # convert the image to grayscale
    rgb2gray = imgproc.cvtColor('rgb -> gray', flag=imgproc.Conversion.RGB2GRAY)
    graph += [source['image'] >> rgb2gray[:] ]

    # Find planes
    plane_est = PlaneFinder(min_size=10000)
    compute_normals = ComputeNormals()
    graph += [ # find the normals
                source['K', 'points3d'] >> compute_normals['K', 'points3d'],
                # find the planes
                compute_normals['normals'] >> plane_est['normals'],
                source['K', 'points3d'] >> plane_est['K', 'points3d'] ]

    display = highgui.imshow(name='Poses')
    if orb_template:
        # find the pose using ORB
        poser = OrbPoseEstimator(directory=orb_template, show_matches=orb_matches)
        graph += [ source['image', 'K', 'mask_depth', 'points3d'] >> poser['color_image', 'K', 'mask', 'points3d'],
                   rgb2gray[:] >> poser['image'],
                   poser['debug_image'] >> display['image'],
                 ]
    else:
        # get a pose use the dot pattern: there might be a scale ambiguity as this is 3d only
        poser_tmp = OpposingDotPoseEstimator(rows=5, cols=3,
                                     pattern_type=calib.ASYMMETRIC_CIRCLES_GRID,
                                     square_size=0.04, debug=True)
        graph += [ source['image', 'K'] >> poser_tmp['color_image', 'K'],
                   rgb2gray[:] >> poser_tmp['image'] ]

        # filter the previous pose and resolve the scale ambiguity using 3d
        poser = object_recognition_capture.ecto_cells.capture.PlaneFilter();

        # make sure the pose is centered at the origin of the plane
        graph += [ source['K'] >> poser['K'],
                   poser_tmp['R', 'T'] >> poser['R', 'T'],
                   plane_est['planes', 'masks'] >> poser['planes', 'masks'] ]

        # draw the found pose
        pose_drawer = calib.PoseDrawer('Pose Draw')
        graph += [ poser['found'] >> pose_drawer['trigger'],
                   source['K', 'image'] >> pose_drawer['K', 'image'],
                   poser['R', 'T'] >> pose_drawer['R', 'T'],
                   pose_drawer['output'] >> display[:] ]

    delta_pose = ecto.If('delta R|T', cell=object_recognition_capture.DeltaRT(angle_thresh=angle_thresh,
                                                          n_desired=n_desired))

    poseMsg = RT2PoseStamped(frame_id='/camera_rgb_optical_frame')

    graph += [ poser['R', 'T', 'found'] >> delta_pose['R', 'T', 'found'],
               poser['R', 'T'] >> poseMsg['R', 'T'] ]

    # publish the source data
    rgbMsg = Mat2Image(frame_id='/camera_rgb_optical_frame', swap_rgb=True)
    depthMsg = Mat2Image(frame_id='/camera_rgb_optical_frame')
    graph += [ source['depth'] >> depthMsg[:],
               source['image'] >> rgbMsg[:] ]

    # mask out the object
    masker = segmentation_cell
    graph += [ source['points3d', 'K'] >> masker['points3d', 'K'],
               plane_est['masks', 'planes'] >> masker['masks', 'planes'],
               poser['T'] >> masker['T'] ]

    # publish the mask
    maskMsg = Mat2Image(frame_id='/camera_rgb_optical_frame')
    graph += [ masker['mask'] >> maskMsg[:] ]

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
