import time
from scipy.signal import butter, lfilter
# from importlib import simple
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, Pose
import numpy as np
from cubic_bezier_ import Bezier
from backstepping import Backstepping
import math


class RealScoutControl(Node):
    def __init__(self):
        super().__init__('Control')
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
        self.timer = self.create_timer(self.timer_period, self.scout_control)
        self.start_time = self.get_clock().now().to_msg().sec
        self.start_timet = time.time()
        self.control_points = np.zeros((3, 2))
        self.value_locked = False  # 값이 고정되었는지 여부
        self.x_d = 0
        self.y_d = 0 
        self.x_dp = 0
        self.y_dp = 0 
        self.t_robot = 0
        self.r = 0.165
        self.l = 0.582
        self.Bezier = Bezier()
        self.Backstepping = Backstepping()
        value_locked = False  # 값이 고정되었는지 여부
        self.y_theta_list = []

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
        # Filter requirements.
        cutoff = 5.0  # 저역통과 필터의 컷오프 주파수
        fs = 1/self.timer_period    # 프레임 속도 (초당 프레임)
        order = 3     # 필터 차수
        # 좌표 값 버퍼 크기 조정 (필터링할 데이터 크기 유지)
        if len(self.y_theta_list) > 10:
            self.y_theta_list.pop(0)
        
        # 데이터가 충분할 때 필터 적용
        if len(self.y_theta_list) > order:
            filtered_y = self.butter_lowpass_filter(self.e_theta_list, cutoff, fs, order)
            y = filtered_y

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

    def scout_control(self):
        self.t_robot = time.time()-self.start_timet
        print
        self.x_d, self.y_d, x_dp, y_dp, x_dpp, y_dpp = self.Bezier.bezier_curve(control_points=self.control_points, t_robot=self.t_robot, total_time=30)

        vc, wc = self.Backstepping.backstepping_control(self.x_d, self.y_d, self.x_desired, self.y_desired, x_dp, y_dp, x_dpp, y_dpp, self.y_e, K1=12, K2=3, K3=3)

        vc = float(vc)
        if vc > 0.2:
            vc = 0.2
        elif vc < -0.2:
            vc = -0.2
        wc = float(wc)
        if wc > 0.01:
            wc = 0.01
        elif wc < -0.01:
            wc = -0.01
        if self.x_d > self.control_points[2][0]:
             vc = 0.0
             wc = 0.0
        print(vc,wc)

        twist = Twist()
        twist.linear.x = vc
        twist.angular.z = wc
        self.controlpublisher.publish(twist)


def main(args=None):
    rclpy.init(args=args)
    simple_controller = RealScoutControl()
    rclpy.spin(simple_controller)
    simple_controller.destroy_node()
    rclpy.shutdown()
    
if __name__ == '__main__':
    main()    