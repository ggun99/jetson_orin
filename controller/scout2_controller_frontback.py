import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist, Pose
import numpy as np
import math


### get the position and orientation (x,y,theta) add the offset and move to the position with controller.

class SimpleController(Node):

    def __init__(self):
        super().__init__('current_odom_subscriber')
        self.subscription = self.create_subscription(
            Pose,
            '/aruco_pose',
            self.listener_callback,
            10)
        self.subscription 
        self.K1 = 0.01
        self.K2 = 5.0
        self.controlpublisher = self.create_publisher(Twist,'/cmd_vel', 10)
        self.referencePose = np.array([[5.0],[5.0],[0]])
        self.dTol = 0.01 # distance Tolerance? 
        self.state = False
        self.distance_max = 500 # in mm
        self.distance_min = 450 # in mm

    def euler_from_quaternion(self, x, y, z, w):
        t0 = +2.0 * (w * x + y * z)
        t1 = +1.0 - 2.0 * (x * x + y * y)
        roll_x = math.atan2(t0, t1)

        t2 = +2.0 * (w * y - z * x)
        t2 = +1.0 if t2 > +1.0 else t2
        t2 = -1.0 if t2 < -1.0 else t2
        pitch_y = math.asin(t2)

        t3 = +2.0 * (w * z + x * y)
        t4 = +1.0 - 2.0 * (y * y + z * z)
        yaw_z = math.atan2(t3, t4)

        return roll_x, pitch_y, yaw_z # in radians  

    def kinematic_control(self, x_p, y_p): #, z_e):
        distance = x_p
        if distance > self.distance_max:
            e = distance - self.distance_max
            vc = float(self.K1 * e)
        elif distance < self.distance_min:
            e = distance - self.distance_min
            vc = float(self.K1 * e)
        else:
            vc = float(0.0) 
        # referencePose = self.referencePose
        # currentPose = np.array([[x_p],[y_p],[z_e]])
        
        # controller switch condition
        # if np.linalg.norm([(referencePose[0,0] - currentPose[0,0]),(referencePose[1,0] - currentPose[1,0])]) < self.dTol:
        #     self.state = True
        # # position controller
        # if not self.state:
        #     phiref = np.arctan2((referencePose[1, 0] - currentPose[1, 0]), (referencePose[0, 0] - currentPose[0, 0]))
        #     qRef = np.array([referencePose[0, 0], referencePose[1, 0], phiref]).reshape(3, 1)
        #     e = qRef - currentPose
        #     vc = self.K1 * np.sqrt((e[0, 0]**2) + (e[1, 0]**2))
        #     wc = self.K2 * e[2, 0]
        # orientation controller
        # if self.state:
        #     e = referencePose[2, 0] - currentPose[2, 0]
        #     vc = 0
        #     wc = self.Korient * e
        # print('vc',vc)
        # print('wc',wc)
        
        return vc  # , wc #np.array([[vc], [wc]])

    def listener_callback(self, msg):
        # position
        x_p = msg.position.x
        y_p = msg.position.y
        z_p = msg.position.z
        # quarternion
        # x_q = msg.pose.pose.orientation.x
        # y_q = msg.pose.pose.orientation.y
        # z_q = msg.pose.pose.orientation.z
        # w_q = msg.pose.pose.orientation.w
        # quarternion to euler
        # x, y, z = self.euler_from_quaternion(x_q, y_q, z_q, w_q)
        # euler
        # self.x_e = x
        # self.y_e = y
        # self.z_e = z
        
        vc = self.kinematic_control(x_p, y_p)#, z)
            # vc, wc = self.kinematic_control(x_p, y_p)#, z)
        twist = Twist()
        twist.linear.x = vc
        # twist.angular.z = wc
        
        self.controlpublisher.publish(twist)
    # def get_quaternion_from_euler(self, roll, pitch, yaw):
    #     """
    #     Convert an Euler angle to a quaternion.
        
    #     Input
    #         :param roll: The roll (rotation around x-axis) angle in radians.
    #         :param pitch: The pitch (rotation around y-axis) angle in radians.
    #         :param yaw: The yaw (rotation around z-axis) angle in radians.
        
    #     Output
    #         :return qx, qy, qz, qw: The orientation in quaternion [x,y,z,w] format
    #     """
    #     qx = np.sin(roll/2) * np.cos(pitch/2) * np.cos(yaw/2) - np.cos(roll/2) * np.sin(pitch/2) * np.sin(yaw/2)
    #     qy = np.cos(roll/2) * np.sin(pitch/2) * np.cos(yaw/2) + np.sin(roll/2) * np.cos(pitch/2) * np.sin(yaw/2)
    #     qz = np.cos(roll/2) * np.cos(pitch/2) * np.sin(yaw/2) - np.sin(roll/2) * np.sin(pitch/2) * np.cos(yaw/2)
    #     qw = np.cos(roll/2) * np.cos(pitch/2) * np.cos(yaw/2) + np.sin(roll/2) * np.sin(pitch/2) * np.sin(yaw/2)

    #     return [qx, qy, qz, qw]

  

def main(args = None):
    rclpy.init(args=args)
    simple_controller = SimpleController()
    rclpy.spin(simple_controller)
    simple_controller.destroy_node()
    rclpy.shutdown()
    
if __name__ == '__main__':
    main()      


