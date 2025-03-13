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
        self.K1 = 0.001
        self.K2 = 0.001
        self.controlpublisher = self.create_publisher(Twist,'/cmd_vel', 10)
        self.dTol = 0.05 # distance Tolerance? 
        self.state = False
        self.offset = 0.4
        

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

    def uni_control(self, x_p, y_p, y_e): # all the reference 
        k1 = 1
        k2 = -1
        k3 = -0.5
        ud1 = 2.0
        
        # Calculate errors in position
        theta_ref = y_e
        x_ref = x_p - self.offset
        y_ref = y_p

        z1_ref = theta_ref
        z2_ref = x_ref*np.cos(theta_ref) + y_ref*np.sin(theta_ref)
        z3_ref = x_ref*np.sin(theta_ref) - y_ref*np.cos(theta_ref)

        z1_cur = 0.0
        z2_cur = 0.0
        z3_cur = 0.0
        
        err_z1 = z1_ref
        err_z2 = z2_ref
        err_z3 = z3_ref

        v1 = -k1*err_z1
        v2 = -k2*err_z2 - k3/ud1*err_z3

        u1 = v1*z1_ref + v2
        u2 = v1

        # Send the velocities
        # vmax = 21 , wmax = 5
        vc = float(u1)
        if vc > 0.2:
            vc = 0.2
        elif vc < -0.2:
            vc = -0.2
        wc = float(u2)
        if wc > 0.1:
            wc = 0.1
        elif wc < -0.1:
            wc = -0.1
       
        return vc, wc

    def listener_callback(self, msg):
        # position
        x_p = msg.position.x
        y_p = msg.position.y
        z_p = msg.position.z
        # quarternion
        x_q = msg.orientation.x
        y_q = msg.orientation.y
        z_q = msg.orientation.z
        w_q = msg.orientation.w
        # quarternion to euler
        x, y, z = self.euler_from_quaternion(x_q, y_q, z_q, w_q)
        # euler
        # self.x_e = x
        y_e = y
        # self.z_e = z
        vc, wc = self.uni_control(x_p, y_p, y_e)#, z)

        twist = Twist()
        twist.linear.x = vc
        twist.angular.z = wc
        
        self.controlpublisher.publish(twist)


def main(args = None):
    rclpy.init(args=args)
    simple_controller = SimpleController()
    rclpy.spin(simple_controller)
    simple_controller.destroy_node()
    rclpy.shutdown()
    
if __name__ == '__main__':
    main()      


