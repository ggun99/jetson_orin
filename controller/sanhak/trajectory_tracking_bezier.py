import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist, Pose
import numpy as np
import math
from scipy.signal import butter, lfilter
import cv2 as cv
import time


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
        self.timer_period = 0.05
        self.x_cur = 0.0
        self.y_cur = 0.0
        self.offset = 1.0
        self.x_desired = 0.0
        self.y_desired = 0.0
        self.y_e = 0.0
        self.total_time = 20.0
        self.e_theta_list = []
        self.timer = self.create_timer(self.timer_period, self.generate_bezier_trajectory)
        self.rate = self.create_rate(20) 
        self.start_time = self.get_clock().now().to_msg().sec
        self.curve = np.zeros((1, 2))
        self.curve_v = np.zeros((1, 2))
        self.curve_a = np.zeros((1, 2))
        self.t = 0
        self.control_points = np.zeros((3, 2)) #np.array([[0, 0], [50, 500], [120, 800]])
        self.value_locked = False  # 값이 고정되었는지 여부

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
        # v_c = -v_c
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
        
        # print(f'2: {self.x_start}')
        if not self.value_locked:  # 처음 한 번만 값 저장
            self.x_desired = x_p
            self.y_desired = y_p
            self.control_points[1][0] = self.x_desired*3/5
            self.control_points[1][1] = self.y_desired*4/5
            self.control_points[2][0] = self.x_desired
            self.control_points[2][1] = self.y_desired

            self.value_locked = True
        self.get_logger().info(f"🔒 값 저장: {self.x_desired, self.y_desired}")
        
        
        # euler
        # self.x_e = x
        self.y_e = y
        # self.z_e = z

    def bezier_curve(self, control_points, num_points=400):
        """Calculates a Bezier curve from a set of control points."""
        n = len(control_points) - 1

        for j in range(n + 1):
            binomial_coeff = (
                np.math.factorial(n) / (np.math.factorial(j) * np.math.factorial(n - j))
            )

            # 곡선 위치 계산 (원래 코드)
            self.curve += (
                control_points[j]
                * binomial_coeff
                * self.t ** j
                * (1 - self.t) ** (n - j)
            )

            # 속도(미분) 계산
            if j > 0:  # j=0일 때 0^-1 문제 방지
                self.curve_v += (
                    control_points[j]
                    * binomial_coeff
                    * (j/self.total_time * self.t ** (j-1) * (1 - self.t) ** (n - j) - (n - j)/self.total_time * self.t ** j * (1 - self.t) ** (n - j - 1))
                )

            # self.curve_v += (
            #     control_points[j]
            #     * (
            #         np.math.factorial(n)
            #         / (np.math.factorial(j) * np.math.factorial(n - j))
            #     )
            #     * (j * self.t ** (j-1) * (1 - self.t) ** (n - j) - (n - j) * self.t ** j * (1-self.t) ** (n-j-1))
            # )
        self.t += 1/num_points
        print(self.t)

    def generate_bezier_trajectory(self):
        print(self.control_points)
        self.bezier_curve(self.control_points)
        # time_step = self.timer_period
        start_time = time.time()
        
        # if np.abs(self.x_start) < self.offset:
        #     speed = 0.0
        # else: 
        #     speed = (self.x_start - self.offset) / (self.total_time)  # Aruco~MobileRobot
        x_d = self.curve[0][0]  # x position changes linearly with time
        print(x_d)
        y_d = self.curve[0][1]  # y is constant
        print(y_d)
        x_dp = self.curve_v[0][0]   # Constant speed (dx/dt)
        y_dp = self.curve_v[0][1]   # No change in y (dy/dt)
        x_dpp = 0  # No acceleration in x (d^2x/dt^2)
        y_dpp = 0  # No acceleration in y (d^2y/dt^2)

        vc, wc = self.backstepping_control(self.x_cur, self.y_cur, self.x_desired, self.y_desired, x_dp, y_dp, x_dpp, y_dpp, self.y_e, K1=3.0, K2=1.0, K3=1.0)
        self.x_cur = x_d
        self.y_cur = y_d
        if self.x_cur > self.control_points[2][0]:
            vc = 0 
            wc = 0
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
        print(vc,wc)
        twist = Twist()
        twist.linear.x = vc
        twist.angular.z = wc
        
        self.controlpublisher.publish(twist)
        elapsed_time = time.time() - start_time 
        print(elapsed_time)
        self.rate.sleep()
        return

def main(args = None):
    rclpy.init(args=args)
    simple_controller = SimpleController()
    rclpy.spin(simple_controller)
    simple_controller.destroy_node()
    rclpy.shutdown()
    
if __name__ == '__main__':
    main()      


