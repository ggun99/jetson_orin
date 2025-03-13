
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

    while i < 500:
        # A new image is available if grab() returns SUCCESS
        if zed.grab(runtime_parameters) == sl.ERROR_CODE.SUCCESS:
            # Retrieve left image
            zed.retrieve_image(image, sl.VIEW.LEFT)
            imgnp = image.get_data()
            cv.imshow('img',imgnp)

            if cv.waitKey(1) & 0xFF == ord('q'):
                break

            # # Retrieve depth map. Depth is aligned on the left image
            # zed.retrieve_measure(depth, sl.MEASURE.DEPTH)
            # # Retrieve colored point cloud. Point cloud is aligned on the left image.
            # zed.retrieve_measure(point_cloud, sl.MEASURE.XYZRGBA)

            # # Get and print distance value in mm at the center of the image
            # # We measure the distance camera - object using Euclidean distance
            # x = round(image.get_width() / 2)
            # y = round(image.get_height() / 2)
            # err, point_cloud_value = point_cloud.get_value(x, y)

            # if math.isfinite(point_cloud_value[2]):
            #     distance = math.sqrt(point_cloud_value[0] * point_cloud_value[0] +
            #                         point_cloud_value[1] * point_cloud_value[1] +
            #                         point_cloud_value[2] * point_cloud_value[2])
            #     print(f"Distance to Camera at {{{x};{y}}}: {distance}")
            # else : 
            #     print(f"The distance can not be computed at {{{x};{y}}}")
            # i += 1    
           

    # Close the camera
    zed.close()

if __name__ == "__main__":
    main()