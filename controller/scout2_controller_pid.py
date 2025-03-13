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
        self.offset = 500
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
        # Calculate errors in position
        distance = x_p
        e_x = distance - self.offset
        ro_distance = y_p
        e_y = ro_distance

        integral_dist = 0.0
        previous_err_dist = 0.0
        integral_theta = 0.0
        previous_err_theta = 0.0
        
        err_dist = e_x
        
        # Error in heading
        err_theta = e_y
       
        
        Kp_dist = 0.00001
        Ki_dist = 0.01
        Kd_dist = 0.08
        Kp_theta = 0.0001
        Ki_theta = 0.01
        Kd_theta = 0.01
        
        
        
        integral_dist += err_dist
        derivative_dist = err_dist - previous_err_dist
        integral_theta += err_theta
        derivative_theta = err_theta - previous_err_theta
        

        # TODO: Add integral and derivative calculations for complete PID

        # PID control for linear velocity
        
        if np.abs(err_dist) >= self.dTol: #checking whether error distance within tolerence
            l_v = Kp_dist * abs(err_dist) + Ki_dist * integral_dist + Kd_dist * derivative_dist
            previous_err_dist = err_dist
        else:
            self.get_logger().info(f"Scout2.0  stopping goal distance within tolerence")
            l_v = 0.0    


        # PID control for angular velocity
        if np.abs(err_theta) >= self.dTol: #checking whether heading angle error within tolerence
            a_v = Kp_theta * err_theta + Ki_theta * integral_theta + Kd_theta * derivative_theta
            previous_err_theta = err_theta
            
        else:
            self.get_logger().info(f"Scout2.0  stopping goal heading within tolerence")
            a_v = 0.0      

        # Send the velocities
        # vmax = 21 , wmax = 5
        vc = float(l_v)
        if vc > 0.2:
            vc = 0.2
        elif vc < -0.2:
            vc = -0.2
        wc = float(a_v)
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

        vc, wc = self.kinematic_control(x_p, y_p)#, z)

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


