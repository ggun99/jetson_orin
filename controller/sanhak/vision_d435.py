import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image as msg_Image
from sensor_msgs.msg import CameraInfo
from cv_bridge import CvBridge, CvBridgeError
import cv2
import sys
import os
import numpy as np
import pyrealsense2 as rs2
from geometry_msgs.msg import Pose
import tf2_ros

if (not hasattr(rs2, 'intrinsics')):
    import pyrealsense2.pyrealsense2 as rs2

class ImageListener(Node):
    def __init__(self, depth_image_topic):
        super().__init__('listener')
        self.position = None
        self.sub_img = self.create_subscription(msg_Image, '/color_frames', self.find_aruco, 10)
        self.bridge = CvBridge()
        self.sub = self.create_subscription(msg_Image, depth_image_topic, self.imageDepthCallback, 10)
        # self.sub_info = self.create_subscription(CameraInfo, depth_info_topic, self.imageDepthInfoCallback, 1)
        # confidence_topic = depth_image_topic.replace('depth', 'confidence')
        # self.sub_conf = self.create_subscription(msg_Image, confidence_topic, self.confidenceCallback, 1)
        self.intrinsics = None
        self.pix = [0,0]
        self.pix_grade = None
        self.pose_publisher = self.create_publisher(Pose,'/aruco_pose', 10)
        self.tfBuffer = tf2_ros.Buffer()
        self.listener = tf2_ros.TransformListener(self.tfBuffer, self)
        # RealSense 파이프라인 시작
        self.pipeline = rs2.pipeline()
        self.config = rs2.config()
        self.config.enable_stream(rs2.stream.depth, 640, 480, rs2.format.z16, 30)  # Depth 스트림 활성화
        self.config.enable_stream(rs2.stream.color, 640, 480, rs2.format.bgr8, 30)  # Color 스트림 활성화
        # 파이프라인 시작
        self.pipeline_profile = self.pipeline.start(self.config)
        self.depth_sensor = self.pipeline_profile.get_device().first_depth_sensor()
        self.depth_scale = self.depth_sensor.get_depth_scale()
        # Depth 카메라의 프로파일 가져오기
        self.depth_stream = self.pipeline_profile.get_stream(rs2.stream.depth).as_video_stream_profile()
        self.intrinsics = self.depth_stream.get_intrinsics()

    def find_aruco(self, data):
        imgnp = self.bridge.imgmsg_to_cv2(data)
        imgnp = cv2.cvtColor(imgnp, cv2.COLOR_BGR2RGB)
        # Convert the image to grayscale
        gray = cv2.cvtColor(imgnp, cv2.COLOR_RGB2GRAY)

        # Define the dictionary and parameters
        aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_6X6_250)
        parameters = cv2.aruco.DetectorParameters()

        # Create the ArUco detector
        detector = cv2.aruco.ArucoDetector(aruco_dict, parameters)

        # Detect the markers
        corners, ids, rejected = detector.detectMarkers(gray)
        print("Detected markers:", ids)
        if ids is not None:
            middle_point_x = int((corners[0][0][0][0]+corners[0][0][1][0]+corners[0][0][2][0]+corners[0][0][3][0])/4)
            middle_point_y = int((corners[0][0][0][1]+corners[0][0][1][1]+corners[0][0][2][1]+corners[0][0][3][1])/4)
            cv2.circle(imgnp,(middle_point_x,middle_point_y),4,[0,255,0],2)

            # Get and print distance value in mm at the center of the image
            # We measure the distance camera - object using Euclidean distance
            self.pix[0] = middle_point_x
            self.pix[1] = middle_point_y
            # cv2.aruco.drawDetectedMarkers(imgnp, corners, ids)
            # cv2.imshow('Detected Markers', imgnp)   
            # cv2.waitKey(1)
            self.find_aruco = True
        else:
            self.pix[0] = round(imgnp.shape[0] / 2)
            self.pix[0] = round(imgnp.shape[1] / 2)
            self.find_aruco = False
        # err, point_cloud_value = self.point_cloud.get_value(x, y)
        
        # if math.isfinite(point_cloud_value[2]):
        #     distance = math.sqrt(point_cloud_value[0] * point_cloud_value[0] +
        #                         point_cloud_value[1] * point_cloud_value[1] +
        #                         point_cloud_value[2] * point_cloud_value[2])
        #     X = distance*(x-640)/self.focal_left_x
        #     Y = distance*(y-360)/self.focal_left_y
            
        #     print(f"Distance to Camera at {{{x};{y}}}: {distance} mm")
        #     print(f"3D position to Camera : X : {X}, Y : {Y}, Z : {distance}")
        #     self.position = np.array([X,Y,distance])
        # else : 
        #     print(f"The distance can not be computed at {{{x};{y}}}")
        #     self.position = None

    def imageDepthCallback(self, data):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(data, data.encoding)
            pix = (self.pix[0], self.pix[1])
            # self.pix = pix
            line = 'Depth at pixel(%3d, %3d): %7.1f(mm).' % (pix[0], pix[1], cv_image[pix[1], pix[0]])
            if self.intrinsics is not None:
                depth = cv_image[pix[1], pix[0]]
                result = rs2.rs2_deproject_pixel_to_point(self.intrinsics, [pix[0], pix[1]], depth)
                self.position = np.array(result)
                a_pose = Pose()
                # offset = 50  # in mm
                # print(self.H_s2a)
                a_pose.position.x = float(self.position[0])# + offset)
                a_pose.position.y = float(self.position[1])
                a_pose.position.z = float(self.position[2])
                self.pose_publisher.publish(a_pose)

                line += '  Coordinate: %8.2f %8.2f %8.2f.' % (result[0], result[1], result[2])
            if (not self.pix_grade is None):
                line += ' Grade: %2d' % self.pix_grade
            line += '\r'
            sys.stdout.write(line)
            sys.stdout.flush()

        except CvBridgeError as e:
            print(e)
            return
        except ValueError as e:
            return

    def confidenceCallback(self, data):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(data, data.encoding)
            grades = np.bitwise_and(cv_image >> 4, 0x0f)
            if (self.pix):
                self.pix_grade = grades[self.pix[1], self.pix[0]]
        except CvBridgeError as e:
            print(e)
            return
        
    def sub_callback(self, msg):
        self.pix = msg.data



    # def imageDepthInfoCallback(self, cameraInfo):
    #     try:
    #         # RealSense 파이프라인 시작
    #         pipeline = rs2.pipeline()
    #         config = rs2.config()
    #         config.enable_stream(rs2.stream.depth, 640, 480, rs2.format.z16, 30)  # Depth 스트림 활성화
    #         config.enable_stream(rs2.stream.color, 640, 480, rs2.format.bgr8, 30)  # Color 스트림 활성화

    #         # 파이프라인 시작
    #         pipeline_profile = pipeline.start(config)
    #         depth_sensor = pipeline_profile.get_device().first_depth_sensor()
    #         self.depth_scale = depth_sensor.get_depth_scale()
    #         # Depth 카메라의 프로파일 가져오기
    #         depth_stream = pipeline_profile.get_stream(rs2.stream.depth).as_video_stream_profile()
    #         # if self.intrinsics:
    #         #     return
    #         self.intrinsics = depth_stream.get_intrinsics()
    #         # if cameraInfo.distortion_model == 'plumb_bob':
    #         #     self.intrinsics.model = rs2.distortion.brown_conrady
    #         # elif cameraInfo.distortion_model == 'equidistant':
    #         #     self.intrinsics.model = rs2.distortion.kannala_brandt4
    #         # self.intrinsics.coeffs = [i for i in cameraInfo.d]
    #     except CvBridgeError as e:
    #         print(e)
    #         return

def main(args=None):
    rclpy.init(args=args)
    depth_image_topic = '/depth_frames'
    # depth_info_topic = '/camera/aligned_depth_to_color/camera_info'

    # print ()
    # print ('show_center_depth.py')
    # print ('--------------------')
    # print ('App to demontrate the usage of the /camera/depth topics.')
    # print ()
    # print ('Application subscribes to %s and %s topics.' % (depth_image_topic, depth_info_topic))
    # print ('Application then calculates and print the range to the closest object.')
    # print ('If intrinsics data is available, it also prints the 3D location of the object')
    # print ('If a confedence map is also available in the topic %s, it also prints the confidence grade.' % depth_image_topic.replace('depth', 'confidence'))
    # print ()
    listener = ImageListener(depth_image_topic)
    rclpy.spin(listener)
    listener.destroy_node()
    rclpy.shutdown()    

if __name__ == '__main__':
    main()