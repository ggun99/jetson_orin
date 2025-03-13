
import pyzed.sl as sl
import math
import numpy as np
import math
import cv2 as cv

def main():
    
    # Create a Camera object
    zed = sl.Camera()

    # Create a InitParameters object and set configuration parameters
    init_params = sl.InitParameters()
    init_params.depth_mode = sl.DEPTH_MODE.ULTRA  # Use ULTRA depth mode
    init_params.coordinate_units = sl.UNIT.MILLIMETER  # Use meter units (for depth measurements)

    # Open the camera
    status = zed.open(init_params)
    if status != sl.ERROR_CODE.SUCCESS: #Ensure the camera has opened succesfully
        print("Camera Open : "+repr(status)+". Exit program.")
        exit()

    # Create and set RuntimeParameters after opening the camera
    runtime_parameters = sl.RuntimeParameters()
    
    i = 0
    image = sl.Mat(zed.get_camera_information().camera_configuration.resolution.width, zed.get_camera_information().camera_configuration.resolution.height, sl.MAT_TYPE.U8_C4)
    depth = sl.Mat()
    point_cloud = sl.Mat()

    mirror_ref = sl.Transform()
    mirror_ref.set_translation(sl.Translation(2.75,4.0,0))
    # fx, fy
    focal_left_x = zed.get_camera_information().camera_configuration.calibration_parameters.left_cam.fx
    focal_left_y = zed.get_camera_information().camera_configuration.calibration_parameters.left_cam.fy
    print('fx,fy:', focal_left_x,focal_left_y)

    while True:
        # A new image is available if grab() returns SUCCESS
        if zed.grab(runtime_parameters) == sl.ERROR_CODE.SUCCESS:
            # Retrieve left image
            zed.retrieve_image(image, sl.VIEW.LEFT)
            imgnp = image.get_data()
            # cv.imshow('img',imgnp)
            if imgnp is None:
                raise ValueError("Image not loaded. Please check the image path or URL.")

            # Check the number of channels in the image
            if len(imgnp.shape) == 2:  # Grayscale image
                imgnp = cv.cvtColor(imgnp, cv.COLOR_GRAY2BGR)
            elif imgnp.shape[2] == 4:  # RGBA image
                imgnp = cv.cvtColor(imgnp, cv.COLOR_RGBA2RGB)

            # Convert the image to grayscale
            gray = cv.cvtColor(imgnp, cv.COLOR_BGR2GRAY)

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
                # zed.retrieve_measure(depth, sl.MEASURE.DEPTH)
                # Retrieve colored point cloud. Point cloud is aligned on the left image.
                zed.retrieve_measure(point_cloud, sl.MEASURE.XYZRGBA)

                # Get and print distance value in mm at the center of the image
                # We measure the distance camera - object using Euclidean distance
                x = middle_point_x#round(image.get_width() / 2)
                y = middle_point_y#round(image.get_height() / 2)
            else:
                zed.retrieve_measure(point_cloud, sl.MEASURE.XYZRGBA)
                x = round(image.get_width() / 2)
                y = round(image.get_height() / 2)
                print(x,y)
            err, point_cloud_value = point_cloud.get_value(x, y)
            
            if math.isfinite(point_cloud_value[2]):
                distance = math.sqrt(point_cloud_value[0] * point_cloud_value[0] +
                                    point_cloud_value[1] * point_cloud_value[1] +
                                    point_cloud_value[2] * point_cloud_value[2])
                X = distance*(x-640)/focal_left_x
                Y = distance*(y-360)/focal_left_y
                print(f"Distance to Camera at {{{x};{y}}}: {distance} mm")
                print(f"3D position to Camera : X : {X}, Y : {Y}, Z : {distance}")
            else : 
                print(f"The distance can not be computed at {{{x};{y}}}")
            # i += 1    
            if ids is not None:
                cv.aruco.drawDetectedMarkers(imgnp, corners, ids)
                cv.imshow('Detected Markers', imgnp)
                
            if cv.waitKey(1) & 0xFF == ord('q'):
                break

    # Close the camera
    zed.close()

if __name__ == "__main__":
    main()