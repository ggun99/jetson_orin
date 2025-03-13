import time
import sys
import os
from turtle import distance
import scipy.linalg

import rclpy.time
wd = os.path.abspath(os.getcwd())
sys.path.append(str(wd))

from cv2 import waitKey
import numpy as np
import rtde_control
import rtde_receive
from jacobian_v1 import Jacobian
from admittance_controller_v1 import AdmittanceController
import rclpy
from rclpy.node import Node 
from scipy.signal import butter, lfilter
import threading
from geometry_msgs.msg import Twist, WrenchStamped
import math

class UR5e_controller(Node):
    def __init__(self):
        super().__init__('UR5e_node')
        self.ROBOT_IP = '192.168.0.212'  # 로봇의 IP 주소로 바꿔주세요
        # RTDE 수신 객체 생성
        self.rtde_r = rtde_receive.RTDEReceiveInterface(self.ROBOT_IP)
        # RTDE Control Interface 초기화
        self.rtde_c = rtde_control.RTDEControlInterface(self.ROBOT_IP)
        self.dTol = 0.005
        self.derivative_dist = 0.0
        self.maxForce = 100
        self.integral_dist = 0.0
        self.previous_err_dist = 0.0
        self.integral_theta = 0.0
        self.previous_err_theta = 0.0
        self.state = np.array([0, 0, 0, 0, 0, 0]).reshape(-1, 1)
        self.U_previous = None
        self.Sigma_previous = None
        self.Vt_previous = None  
        self.previous_time = time.time() 
        self.Jacobian = Jacobian()
        M = np.diag([25.0, 25.0, 25.0, 0.3, 0.3, 0.3])  # Mass matrix
        B = np.diag([140.0, 140.0, 140.0, 2.0, 2.0, 2.0])  # Damping matrix
        K = np.diag([200.0, 200.0, 200.0, 0.0, 0.0, 0.0])  # Stiffness matrix
        self.admit = AdmittanceController(M, B, K)
        self.dt = 0.05
        self.timer = self.create_timer(self.dt, self.admittance)
         # ft data list 저장
        self.ft_x = []
        self.ft_y = []
        self.ft_z = []
        self.emergency_stop = False
        # Start a thread to listen for emergency stop input
        self.emergency_stop_thread = threading.Thread(target=self.emergency_stop_listener)
        self.emergency_stop_thread.daemon = True
        self.emergency_stop_thread.start()
        # mobile robot
        self.dTol = 0.05 # distance Tolerance? 
        self.controlpublisher = self.create_publisher(Twist,'/cmd_vel', 10)
        self.ftpublisher = self.create_publisher(WrenchStamped,'/ur_ftsensor', 10)
        self.K1 = 0.2
        self.K2 = 0.2
        self.integral_dist = 0
        self.previous_err_x = 0
        self.previous_err_y = 0

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

    def kinematic_control(self, e_x, e_y):
        Kp_x = 0.05
        Ki_x = 0.1
        Kd_x = 0.08
        Kp_y = 0.8
        Ki_y = 0.1
        Kd_y = 0.01
        
        self.integral_dist += e_x
        # Prevent integral windup
        self.integral_dist = min(max(self.integral_dist, -10), 10)
        
        derivative_dist = e_x - self.previous_err_x
        derivative_theta = e_y - self.previous_err_y
        vc = Kp_x * abs(e_x) + Ki_x * self.integral_dist + Kd_x * derivative_dist
        if -self.dTol < vc < self.dTol:
            vc = 0.0
            print("Scout2.0 stopping - distance within tolerance")
            self.previous_err_x = 0.0
        elif vc > 0.1 :
            vc = 0.1
            self.previous_err_x = e_x
        elif vc < -0.1 :
            vc = -0.1
            self.previous_err_x = e_x
        else:
            vc = vc

        wc = Kp_y * abs(e_y) + Ki_y * self.integral_dist + Kd_y * derivative_theta
        if -self.dTol < wc < self.dTol:
            # wc = Kp_y * abs(e_y) + Ki_y * self.integral_dist + Kd_y * derivative_dist
            wc = 0.0
            print("Scout2.0 stopping - distance within tolerance")
            self.previous_err_y = 0.0
        elif wc > 0.1 :
            wc = 0.1
            self.previous_err_y = e_y
        elif wc < -0.1 :
            wc = -0.1
            self.previous_err_y = e_y
        else:
            wc = wc 

       
        return vc, wc #np.array([[vc], [wc]])

    def emergency_stop_listener(self):
        #  """Listen for Enter key to activate emergency stop."""
        print("Press Enter to activate emergency stop.")
        while True:
            input()  # Wait for Enter key press
            self.emergency_stop = True
            self.get_logger().warn("Emergency stop activated!")

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
    
    def admittance(self):
        # self.rtde_c.set_gravity([0, 0, -9.81])
        t_start = self.rtde_c.initPeriod()
        wrench = self.rtde_r.getActualTCPForce()   # 중력 혹은 다른 힘들이 보정이 된 TCP 에서 측정된 힘
        # TCPpose = self.rtde_r.getActualTCPPose()
        # TCPpose = np.array(TCPpose)
        # raw_wrench = self.rtde_r.getFtRawWrench()   # 중력 혹은 다른 힘들이 일체 보정이 되지 않은 raw 데이터
        # 6D 벡터: [Fx, Fy, Fz, Tx, Ty, Tz]
        # print("Force/Torque values:", wrench)
        # print("Raw Force/Torque values:", raw_wrench)
        ft = wrench
        ft[0] = ft[0]#+10.0
        ft[3], ft[4], ft[5] = 0, 0, 0
        if not self.rtde_r.isConnected():
            self.rtde_r.reconnect()
        if ft:
            # Filter requirements.
            cutoff = 3.0  # 저역통과 필터의 컷오프 주파수
            fs = 100.0     # 프레임 속도 (초당 프레임)
            order = 3     # 필터 차수
            self.ft_x.append(ft[0])
            self.ft_y.append(ft[1])
            self.ft_z.append(ft[2])
            if len(self.ft_x) > 50 :
                self.ft_x.pop(0)
                self.ft_y.pop(0)
                self.ft_z.pop(0)
            # 데이터가 충분할 때 필터 적용
            if len(self.ft_x) > order:
                filtered_ft_x = self.butter_lowpass_filter(self.ft_x, cutoff, fs, order)
                filtered_ft_y = self.butter_lowpass_filter(self.ft_y, cutoff, fs, order)
                filtered_ft_z = self.butter_lowpass_filter(self.ft_z, cutoff, fs, order)
                ft[0] = filtered_ft_x[-1]
                ft[1] = filtered_ft_y[-1]
                ft[2] = filtered_ft_z[-1]
        t = time.time()

        
        delta_position, _ = self.admit.update(ft, self.dt)
        joint_positions = self.rtde_r.getActualQ()
        wrenchstamp = WrenchStamped()
        wrenchstamp.wrench.force.x = joint_positions[0]
        wrenchstamp.wrench.force.y = joint_positions[1]
        wrenchstamp.wrench.force.z = joint_positions[2]
        self.ftpublisher.publish(wrenchstamp)

        # T = self.admit.forward_kinematics_ur5e(joint_positions)
        # R = T[:3,:3]


        # joint_vel = self.rtde_r.getActualQd()

        # joint velocity value
        # joint_vel_arr = np.array([joint_vel[0],joint_vel[1],joint_vel[2],joint_vel[3],joint_vel[4],joint_vel[5]])
    
        # new jac
        J = self.Jacobian.jacobian(joint_positions[0], joint_positions[1], joint_positions[2], joint_positions[3], joint_positions[4], joint_positions[5])
        j_J = J #@ J.T
        
        # SVD 계산
        try:
            U, Sigma, Vt = np.linalg.svd(j_J, full_matrices=False, hermitian=False)
            self.U_previous, self.Sigma_previous, self.Vt_previous = U, Sigma, Vt  # 이전 값 업데이트
        except ValueError as e:
            print("SVD computation failed due to invalid input:")
            print(e)
            U, Sigma, Vt = self.U_previous, self.Sigma_previous, self.Vt_previous  # 이전 값 사용
        
        max_q_dot = 0.1 # 최대 속도 한계를 설정

        # pseudo inverse jacobian matrix
        J_pseudo_inv = np.linalg.pinv(J)
        print(f"==>> J_pseudo_inv: {J_pseudo_inv}")

        d_goal_v = delta_position
        print(f"==>> delta_position: {delta_position}")
        print(f"==>> d_goal_v: {d_goal_v}")
        q_dot = J_pseudo_inv @ d_goal_v

        cal_check = J @ q_dot
        print('checking:',cal_check)
        # 속도가 한계를 초과하면 제한
        q_dot = np.clip(q_dot, -max_q_dot, max_q_dot)
        q_dot = q_dot.flatten()
        acceleration = 0.2


        if self.emergency_stop:
            q_dot = np.array([[0.0],[0.0],[0.0],[0.0],[0.0],[0.0]])
        # control manipulator
        self.rtde_c.speedJ(q_dot, acceleration, self.dt)
        self.rtde_c.waitPeriod(t_start)
        # control mobile robot
        vc, wc = self.kinematic_control(delta_position[0], delta_position[1])
        print(f"==>> wc: {wc}")
        print(f"==>> vc: {vc}")
        twist = Twist()
        twist.linear.x = vc
        twist.angular.z = wc
        self.controlpublisher.publish(twist)

def main(args=None):
    rclpy.init(args=args)
    listener = UR5e_controller()
    rclpy.spin(listener)
    listener.destroy_node()
    rclpy.shutdown() 
    listener.rtde_c.servoStop()
    listener.rtde_c.stopScript()


print('Program started')

if __name__ == '__main__':
    waitKey(3000)
    main()