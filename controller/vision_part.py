import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Pose
import pyzed.sl as sl
import math
import numpy as np
import math
import cv2 as cv
from cv_bridge import CvBridge

class Vision_Tracker(Node):
    def __init__(self):
        super().__init__('Vision_Tracker')
        self.position = None
        self.H_s2a = None
        self.pose_publisher = self.create_publisher(Pose,'/aruco_pose', 10)
        self.H_s2a = None
        self.timeroffset = self.create_timer(0.1, self.offset_pub)
        # Create a Camera object
        self.zed = sl.Camera()
        # Create a InitParameters object and set configuration parameters
        init_params = sl.InitParameters()
        init_params.depth_mode = sl.DEPTH_MODE.ULTRA  # Use ULTRA depth mode
        init_params.coordinate_units = sl.UNIT.MILLIMETER  # Use meter units (for depth measurements)    
        # Open the camera   
        status = self.zed.open(init_params)
        if status != sl.ERROR_CODE.SUCCESS: #Ensure the camera has opened succesfully
            print("Camera Open : "+repr(status)+". Exit program.")
        # Create and set RuntimeParameters after opening the camera
        self.runtime_parameters = sl.RuntimeParameters()
        self.image = sl.Mat(self.zed.get_camera_information().camera_configuration.resolution.width, self.zed.get_camera_information().camera_configuration.resolution.height, sl.MAT_TYPE.U8_C4)
        self.depth = sl.Mat()
        self.point_cloud = sl.Mat()

        mirror_ref = sl.Transform()
        mirror_ref.set_translation(sl.Translation(2.75,4.0,0))
        # fx, fy
        self.focal_left_x = self.zed.get_camera_information().camera_configuration.calibration_parameters.left_cam.fx
        self.focal_left_y = self.zed.get_camera_information().camera_configuration.calibration_parameters.left_cam.fy
        self.bridge = CvBridge()
   

    def Image_Processing(self):
        
        
        # while True:
        # A new image is available if grab() returns SUCCESS
        if self.zed.grab(self.runtime_parameters) == sl.ERROR_CODE.SUCCESS:
            # Retrieve left image
            self.zed.retrieve_image(self.image, sl.VIEW.LEFT)
            imgnp = self.image.get_data()
            # cv.imshow('img',imgnp)
            if imgnp is None:
                raise ValueError("Image not loaded. Please check the image path or URL.")

            # Check the number of channels in the image
            if len(imgnp.shape) == 2:  # Grayscale image
                imgnp = cv.cvtColor(imgnp, cv.COLOR_GRAY2BGR)
            elif imgnp.shape[2] == 4:  # RGBA image
                imgnp = cv.cvtColor(imgnp, cv.COLOR_RGBA2RGB)

            # Convert the image to grayscale
            gray = cv.cvtColor(imgnp, cv.COLOR_RGB2GRAY)

            # Define the dictionary and parameters
            aruco_dict = cv.aruco.getPredefinedDictionary(cv.aruco.DICT_6X6_250)
            parameters = cv.aruco.DetectorParameters()

            # Create the ArUco detector
            detector = cv.aruco.ArucoDetector(aruco_dict, parameters)

            # Detect the markers
            corners, ids, rejected = detector.detectMarkers(gray)
            print("Detected markers:", ids)
            if ids is not None:
                middle_point_x = int((corners[0][0][0][0]+corners[0][0][1][0]+corners[0][0][2][0]+corners[0][0][3][0])/4)
                middle_point_y = int((corners[0][0][0][1]+corners[0][0][1][1]+corners[0][0][2][1]+corners[0][0][3][1])/4)
                cv.circle(imgnp,(middle_point_x,middle_point_y),4,[0,255,0],2)

                # Retrieve depth map. Depth is aligned on the left image
                self.zed.retrieve_measure(self.depth, sl.MEASURE.DEPTH)
                # Retrieve colored point cloud. Point cloud is aligned on the left image.
                self.zed.retrieve_measure(self.point_cloud, sl.MEASURE.XYZRGBA)

                # Get and print distance value in mm at the center of the image
                # We measure the distance camera - object using Euclidean distance
                x = middle_point_x
                y = middle_point_y
                
            else:
                self.zed.retrieve_measure(self.point_cloud, sl.MEASURE.XYZRGBA)
                x = round(self.image.get_width() / 2)
                y = round(self.image.get_height() / 2)
            cv.aruco.drawDetectedMarkers(imgnp, corners, ids)
            cv.imshow('Detected Markers', imgnp)   
            cv.waitKey(1)    
            err, point_cloud_value = self.point_cloud.get_value(x, y)
            
            if math.isfinite(point_cloud_value[2]):
                distance = math.sqrt(point_cloud_value[0] * point_cloud_value[0] +
                                    point_cloud_value[1] * point_cloud_value[1] +
                                    point_cloud_value[2] * point_cloud_value[2])
                X = distance*(x-640)/self.focal_left_x
                Y = distance*(y-360)/self.focal_left_y
                
                print(f"Distance to Camera at {{{x};{y}}}: {distance} mm")
                print(f"3D position to Camera : X : {X}, Y : {Y}, Z : {distance}")
                self.position = np.array([X,Y,distance])
            else : 
                print(f"The distance can not be computed at {{{x};{y}}}")
                self.position = None
                
    
    def Transform(self):
        if self.position is not None:
            # aruco based on camera
            H_a2c = np.array([[1,0,0,self.position[0]],[0,1,0,self.position[1]],[0,0,1,self.position[2]],[0,0,0,1]])

            # camera to based on scout
            P_x = 0
            P_y = 0
            P_z = 0

            H_s2c = np.array([[0,0,1,P_x],[-1,0,0,P_y],[0,-1,0,P_z],[0,0,0,1]])

            # aruco based on scout
            self.H_s2a = H_s2c @ H_a2c

        else:
            self.H_s2a = np.array([[1,0,0,0],[0,1,0,0],[0,0,1,0],[0,0,0,1]])
    
    def offset_pub(self):
        self.Image_Processing()
        self.Transform()
        # if H_matrix is not None:
        a_pose = Pose()
        offset = 50  # in mm
        # print(self.H_s2a)
        a_pose.position.x = float(self.H_s2a[0][3])# + offset)
        a_pose.position.y = float(self.H_s2a[1][3])
        a_pose.position.z = float(self.H_s2a[2][3])
        self.pose_publisher.publish(a_pose)
    
def main(args = None):
    rclpy.init(args=args)
    vision_tracker = Vision_Tracker()
    rclpy.spin(vision_tracker)
    vision_tracker.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()