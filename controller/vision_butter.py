import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Pose,Twist
import pyzed.sl as sl
import math
import numpy as np
import math
import cv2 as cv
from scipy.signal import butter, lfilter
import matplotlib.pyplot as plt

class Vision_Tracker(Node):
    def __init__(self):
        super().__init__('Vision_Tracker')
        self.position = False
        self.H_s2a = None
        self.pose_publisher = self.create_publisher(Pose,'/aruco_pose', 10)
        self.H_s2a = None
        self.timeroffset = self.create_timer(0.01667, self.offset_pub)
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
        self.aruco = False
        mirror_ref = sl.Transform()
        mirror_ref.set_translation(sl.Translation(2.75,4.0,0))
        # fx, fy
        self.focal_left_x = self.zed.get_camera_information().camera_configuration.calibration_parameters.left_cam.fx
        self.focal_left_y = self.zed.get_camera_information().camera_configuration.calibration_parameters.left_cam.fy
        self.prev_x = 0.0
        self.prev_y = 0.0
        self.prev_z = 0.0
        self.filter = 50
   
    # Butterworth 저역통과 필터 설계
    def butter_lowpass(self, cutoff, fs, order=5):
        nyq = 0.5 * fs
        normal_cutoff = cutoff / nyq
        b, a = butter(order, normal_cutoff, btype='low', analog=False)
        return b, a

    # 필터 적용 함수
    def butter_lowpass_filter(self, data, cutoff, fs, order=5):
        b, a = self.butter_lowpass(cutoff, fs, order=order)
        y = lfilter(b, a, data)
        return y

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
            # print("Detected markers:", ids)
            data_x = []
            data_y = []
            # Filter requirements.
            cutoff = 5.0  # 저역통과 필터의 컷오프 주파수
            fs = 60.0     # 프레임 속도 (초당 프레임)
            order = 3     # 필터 차수
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
                
                data_x.append(middle_point_x)
                data_y.append(middle_point_y)
                # 좌표 값 버퍼 크기 조정 (필터링할 데이터 크기 유지)
                if len(data_x) > 100:
                    data_x.pop(0)
                    data_y.pop(0)
                x = middle_point_x
                y = middle_point_y
                # 데이터가 충분할 때 필터 적용
                if len(data_x) > order:
                    filtered_x = self.butter_lowpass_filter(data_x, cutoff, fs, order)
                    filtered_y = self.butter_lowpass_filter(data_y, cutoff, fs, order)
                # Filter the data, and plot both the original and filtered signals.
                    cv.circle(imgnp, (int(filtered_x[-1]), int(filtered_y[-1])), 4, [0, 255, 0], 2)
                    self.aruco = True
                    x = filtered_x
                    y = filtered_y
                # cv.aruco.drawDetectedMarkers(imgnp, corners, ids)
                # cv.imshow('Detected Markers', imgnp)   
                # cv.waitKey(1)
            else:
                self.zed.retrieve_measure(self.point_cloud, sl.MEASURE.XYZRGBA)
                self.aruco = False
                x = round(self.image.get_width() / 2)
                y = round(self.image.get_height() / 2)
                
            err, point_cloud_value = self.point_cloud.get_value(x, y)
            
            if math.isfinite(point_cloud_value[2]):
                # distance = math.sqrt(point_cloud_value[0] * point_cloud_value[0] +
                #                     point_cloud_value[1] * point_cloud_value[1] +
                #                     point_cloud_value[2] * point_cloud_value[2])
                distance = point_cloud_value[2]
                X = distance*(x-640)/self.focal_left_x
                Y = distance*(y-360)/self.focal_left_y
                
                # print(f"Distance to Camera at {{{x};{y}}}: {distance} mm")
                print(f"3D position to Camera : X : {X}, Y : {Y}, Z : {distance}")
                self.position = np.array([X,Y,distance])
                self.position = True
            else : 
                print(f"The distance can not be computed at {{{x};{y}}}")
                self.position = False
                
    
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
        if self.position is not False and self.aruco is not False:
            x = self.position[0]
            y = self.position[1]
            z = self.position[2]
            a_pose = Pose()
            
            # print(self.H_s2a)
            # print(x,y,z)
            a_pose.position.x = float(x)
            a_pose.position.y = float(y)
            a_pose.position.z = float(z)
            
            self.pose_publisher.publish(a_pose)

def main(args = None):
    rclpy.init(args=args)
    vision_tracker = Vision_Tracker()
    rclpy.spin(vision_tracker)
    vision_tracker.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()