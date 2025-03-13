import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist, Pose
import numpy as np
import math
from scipy.signal import butter, lfilter


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
        self.controlpublisher = self.create_publisher(Twist,'/cmd_vel', 10)
        self.dTol = 0.05 # distance Tolerance? 
        self.state = False
        self.timer_period = 0.1
        self.x_start = 0.0
        self.offset = 1.0
        self.y_desired = 0.0
        self.y_e = 0.0
        self.total_time = 20.0
        self.e_theta_list = []
        self.timer = self.create_timer(self.timer_period, self.generate_straight_trajectory)
        self.start_time = self.get_clock().now().to_msg().sec

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

    def butter_lowpass(self, cutoff, fs, order=5):
        nyq = 0.5 * fs
        normal_cutoff = cutoff / nyq
        b, a = butter(order, normal_cutoff, btype='low', analog=False)
        return b, a

    # 필터 적용 함수
    def butter_lowpass_filter(self, data, cutoff, fs, order=10):
        b, a = self.butter_lowpass(cutoff, fs, order=order)
        y = lfilter(b, a, data)
        return y

    def backstepping_control(self, x, y, x_d, y_d, x_dp, y_dp, x_dpp, y_dpp, theta, K1=1.0, K2=0.1, K3=1.0):
        e_x = x_d - x
        e_y = y_d - y
        e_theta = np.arctan2(y_dp, x_dp) - theta
        # Filter requirements.
        cutoff = 5.0  # 저역통과 필터의 컷오프 주파수
        fs = 1/self.timer_period    # 프레임 속도 (초당 프레임)
        order = 3     # 필터 차수
        # 좌표 값 버퍼 크기 조정 (필터링할 데이터 크기 유지)
        if len(self.e_theta_list) > 10:
            self.e_theta_list.pop(0)
        
        # 데이터가 충분할 때 필터 적용
        if len(self.e_theta_list) > order:
            filtered_e_theta = self.butter_lowpass_filter(self.e_theta_list, cutoff, fs, order)
            e_theta = filtered_e_theta
        # print('e_theta', e_theta)
        T_e = np.array([[np.cos(theta), np.sin(theta), 0], [-np.sin(theta), np.cos(theta), 0], [0, 0, 1]])
       
        mat_q = T_e @ np.array([[e_x],[e_y],[e_theta]])

        v_r = np.sqrt(x_dp**2+y_dp**2)
        if np.abs(x_dp**2 + y_dp**2) < 0.01:
            w_r = 0.0
        else: 
            w_r = (x_dp*y_dpp - y_dp*x_dpp)/(x_dp**2 + y_dp**2)
        print('mat_q[2,0]', mat_q[2,0])
        print('mat_q[0,0]', mat_q[0,0])
        v_c = v_r*np.cos(mat_q[2,0]) + K1*mat_q[0,0]
        v_c = -v_c
        w_c = w_r + K2*v_r*mat_q[1,0] + K3*np.sin(mat_q[2,0])
     

        return v_c, w_c

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
        # x_start, offset, y_desired, total_time
        # print(f'1: {self.x_start}')
        self.x_start = x_p
        # print(f'2: {self.x_start}')

        self.y_desired = y_p
        
        # euler
        # self.x_e = x
        self.y_e = y
        # self.z_e = z

    def generate_straight_trajectory(self):
        # time_step = self.timer_period
        current_time = self.get_clock().now().to_msg().sec
        elapsed_time = current_time - self.start_time
        if np.abs(self.x_start) < self.offset:
            speed = 0.0
        else: 
            speed = (self.x_start - self.offset) / (self.total_time)  # Aruco~MobileRobot
        x_d = self.x_start - (speed * elapsed_time)  # x position changes linearly with time
        y_d = self.y_desired  # y is constant
        x_dp = speed  # Constant speed (dx/dt)
        y_dp = 0  # No change in y (dy/dt)
        x_dpp = 0  # No acceleration in x (d^2x/dt^2)
        y_dpp = 0  # No acceleration in y (d^2y/dt^2)

        vc, wc = self.backstepping_control(self.x_start, self.y_desired, x_d, y_d, x_dp, y_dp, x_dpp, y_dpp, self.y_e, K1=12.0, K2=5.0, K3=5.0)
        vc = float(vc)
        if vc > 0.2:
            vc = 0.2
        elif vc < -0.2:
            vc = -0.2
        wc = float(wc)
        if wc > 0.1:
            wc = 0.1
        elif wc < -0.1:
            wc = -0.1

        twist = Twist()
        twist.linear.x = vc
        twist.angular.z = wc
        
        self.controlpublisher.publish(twist)
        return

def main(args = None):
    rclpy.init(args=args)
    simple_controller = SimpleController()
    rclpy.spin(simple_controller)
    simple_controller.destroy_node()
    rclpy.shutdown()
    
if __name__ == '__main__':
    main()      


