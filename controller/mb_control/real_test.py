import os
os.environ["OMNI_KIT_ACCEPT_EULA"] = "YES"

from isaacsim import SimulationApp

simulation_app = SimulationApp({"headless": False})


import numpy as np
from scipy.spatial.transform import Rotation as R
from math import atan2 as atan2
import qpsolvers as qp
from spatialmath import base, SE3
import roboticstoolbox as rtb
import matplotlib.pyplot as plt
import random
from cv2 import waitKey
import numpy as np
import rtde_control
import rtde_receive

import rclpy
from rclpy.node import Node 
from geometry_msgs.msg import Twist, Pose
from sensor_msgs.msg import JointState

class QP_mbcontorller(Node):
    def __init__(self):
        super().__init__('mbcontroller')
        self.ROBOT_IP = '192.168.0.212'
        # RTDE 수신 객체 생성
        self.rtde_r = rtde_receive.RTDEReceiveInterface(self.ROBOT_IP)
        # RTDE Control Interface 초기화
        self.rtde_c = rtde_control.RTDEControlInterface(self.ROBOT_IP)
        self.ur5e_robot = rtb.models.UR5()
        self.n_dof = 8 # base(2) + arm(6)
        self.rho_i = 0.9 # influence distance
        self.rho_s = 0.1  # safety factor
        self.eta = 1
        self.qdlim = np.array([0.7]*8)
        self.qdlim[:1] = 1.2  # 베이스 조인트 속도 제한
        self.qdlim[1] = 1.2
        self.qlim = np.array([[-np.inf, -np.inf, -3.14159265, -3.14159265, -3.14159265, -3.14159265, -3.14159265, -3.14159265],
                               [ np.inf, np.inf, 3.14159265,  3.14159265,  3.14159265,  3.14159265,  3.14159265,  3.14159265]])
        self.H_desired = None
        # collision avoidance parameters
        self.d_safe = 0.2
        self.d_influence = 2.0
        self.current_joint_positions = None
        self.q = None
        self.x = 0
        self.y = 0
        self.z = 0
        self.num_points = 10
        self.obstacle_radius = 0.25
        self.lambda_max = 0.32
        self.dt = 0.05
        # self.joint_subscription = self.create_subscription(JointState, '/ur5e_something', self.joint_sub, 10)
        # self.human_position = self.create_subscription(Pose, '/human_position', self.QP_real, 10)
        self.human_position 
        # 장애물의 비콘 위치
        self.obstacles_positions
        # 선의 비콘 위치
        self.points_between
        self.scout_publisher = self.create_publisher(Twist, '/scout_vel', 10)
        # self.ur5e_publisher = self.create_publisher(JointState, 'ur5e_vel', 10)

    def joint_velocity_damper(self, 
            ps: float = 0.05,
            pi: float = 0.1,
            n: int = 8,
            gain: float = 1.0,
        ):
            """
            Compute the joint velocity damper for QP motion control

            Formulates an inequality contraint which, when optimised for will
            make it impossible for the robot to run into joint limits. Requires
            the joint limits of the robot to be specified. See examples/mmc.py
            for use case

            Attributes
            ----------
            ps
                The minimum angle (in radians) in which the joint is
                allowed to approach to its limit
            pi
                The influence angle (in radians) in which the velocity
                damper becomes active
            n
                The number of joints to consider. Defaults to all joints
            gain
                The gain for the velocity damper

            Returns
            -------
            Ain
                A (6,) vector inequality contraint for an optisator
            Bin
                b (6,) vector inequality contraint for an optisator

            """

            Ain = np.zeros((n, n))
            Bin = np.zeros(n)

            for i in range(n):
                if self.q[i] - self.qlim[0, i] <= pi:
                    Bin[i] = -gain * (((self.qlim[0, i] - self.q[i]) + ps) / (pi - ps))
                    Ain[i, i] = -1
                if self.qlim[1, i] - self.q[i] <= pi:
                    Bin[i] = gain * ((self.qlim[1, i] - self.q[i]) - ps) / (pi - ps)
                    Ain[i, i] = 1

            return Ain, Bin

    def get_nearest_obstacle_distance(self, position, obstacles, obstacle_radius, T_e):
        """
        Calculate the distance to the nearest obstacle from a given position in the end-effector frame.
        
        Args:
            position (np.ndarray): The position in world coordinates.
            obstacles (list): A list of obstacle positions in world coordinates.
            obstacle_radius (float): The radius of the obstacles.
            T_cur (np.ndarray): The transformation matrix from world to the robot base.
            T (np.ndarray): The transformation matrix from the robot base to the end-effector.

        Returns:
            float: The distance to the nearest obstacle.
            int: The index of the nearest obstacle.
            np.ndarray: The directional vector to the nearest obstacle in the end-effector frame.
        """
        # 엔드 이펙터의 변환 행렬
        # T_e = T_cur @ T  # 월드 좌표계에서 엔드 이펙터 좌표계로의 변환
        obstacles_local = []
        for obs in obstacles:
            obs[2] = position[2]
            obs_homogeneous = np.append(obs, 1)  # 동차 좌표로 확장
            obs_local = np.linalg.inv(T_e) @ obs_homogeneous
            obstacles_local.append(obs_local[:3])  # 3차원으로 변환
        obstacles_local = np.array(obstacles_local)
        # position을 엔드 이펙터 좌표계로 변환
        
        
        position_homogeneous = np.append(position, 1)  # 동차 좌표로 확장
        position_local = np.linalg.inv(T_e) @ position_homogeneous
        position_local = position_local[:3]  # 3차원으로 변환
        
        distances = [((np.linalg.norm(position_local - obse)) - obstacle_radius) for obse in obstacles_local]
        index = np.argmin(distances)
        # 가장 가까운 장애물에 대한 방향 벡터 계산

        g_vec = (position_local - obstacles_local[index])
        g_vec /= np.linalg.norm(g_vec)  # 방향 벡터 정규화

        return distances, index, g_vec

    def generate_points_between_positions(self, start_pos, end_pos, num_points=10, T_e = None):
        """
        두 3차원 위치를 이어주는 선에서 일정한 간격으로 점을 생성하는 함수.

        Args:
            start_pos (np.ndarray): 시작 위치 (3차원 좌표).
            end_pos (np.ndarray): 끝 위치 (3차원 좌표).
            num_points (int): 생성할 점의 개수 (기본값: 10).

        Returns:
            np.ndarray: 생성된 점들의 좌표 배열 (shape: num_points x 3).
        """
        # position을 엔드 이펙터 좌표계로 변환
        start_pos_local = np.zeros(3)   # 엔드 이펙터 좌표계에서 시작 위치
        # position을 엔드 이펙터 좌표계로 변환
        end_pos_homogeneous = np.append(end_pos, 1)  # 동차 좌표로 확장
        end_pos_local = np.linalg.inv(T_e) @ end_pos_homogeneous
        end_pos_local = end_pos_local[:3]  # 3차원으로 변환

        # 시작 위치와 끝 위치를 연결하는 선을 따라 일정한 간격으로 점 생성
        points = np.linspace(start_pos_local, end_pos_local, num_points)
        dist_vec = (end_pos - start_pos)/num_points

        return points, dist_vec



    # 비콘을 이용한 3차원 위치

    # obstacles_positions = np.array([
    #     [1.2,1.8, 0.97],
    #     [2.8, 0.5, 0.97],
    #     [2.5 , 2.3, 0.97]])


    # # 원기둥 생성
    # obstacle_radius = 0.2
    # obstacle_height = 2.3

    # def joint_sub(self):
    #     # sub the joints values
    #     current_joint_positions = cur_j # 실제 현재 joint 위치
    #     self.current_joint_positions = current_joint_positions
    #     self.x = mobile_base_pose[0][0]
    #     self.y = mobile_base_pose[0][1] 
    #     self.z = mobile_base_pose[0][2] 

    #     quat = mobile_base_quat[0]
    #     self.r = R.from_quat([quat[1], quat[2], quat[3], quat[0]])
    #     self.euler = self.r.as_euler('zyx', degrees=False)  # 'zyx' 순서로 euler angles 추출

    #     self.q = np.zeros(8)
    #     self.q[0] = 0.0
    #     self.q[1] = 0.0 
    #     self.q[2:] = current_joint_positions[4:10]  # UR5e 조인트 위치



    # 여기서 ros를 사용한 것으로 변경
    def QP_real(self):
        t_start = self.rtde_c.initPeriod()
        # sub the joints values
        current_joint_positions = self.rtde_r.getActualQ() # 실제 현재 joint 위치
        self.current_joint_positions = current_joint_positions
        self.x = mobile_base_pose[0][0]
        self.y = mobile_base_pose[0][1] 
        self.z = mobile_base_pose[0][2] 

        quat = mobile_base_quat[0]
        self.r = R.from_quat([quat[1], quat[2], quat[3], quat[0]])
        self.euler = self.r.as_euler('zyx', degrees=False)  # 'zyx' 순서로 euler angles 추출

        self.q = np.zeros(8)
        self.q[0] = 0.0
        self.q[1] = 0.0 
        self.q[2:] = current_joint_positions[4:10]  # UR5e 조인트 위치
        # 3차원 로봇의 링크 위치 입력
        # mobile base links positions and orientations

        rear_right_wheel_pose, rear_right_wheel_quat = prim_rear_right_wheel.get_world_poses()

        rear_left_wheel_pose, rear_left_wheel_quat = prim_rear_left_wheel.get_world_poses()

        front_right_wheel_pose, front_right_wheel_quat = prim_front_right_wheel.get_world_poses()

        front_left_wheel_pose, front_left_wheel_quat = prim_front_left_wheel.get_world_poses()    
        # ur5e joint links positions and orientations

        ur5e_shoulder_pose, ur5e_shoulder_quat = prim_shoulder.get_world_poses()

        ur5e_upper_arm_pose, ur5e_upper_arm_quat = prim_upper_arm.get_world_poses()

        ur5e_forearm_pose, ur5e_forearm_quat = prim_forearm.get_world_poses()

        ur5e_wrist_1_pose, ur5e_wrist_1_quat = prim_wrist_1.get_world_poses()

        ur5e_wrist_2_pose, ur5e_wrist_2_quat = prim_wrist_2.get_world_poses()

        ur5e_wrist_3_pose, ur5e_wrist_3_quat = prim_wrist_3.get_world_poses()

        xform_pose = np.array([rear_right_wheel_pose[0], rear_left_wheel_pose[0], front_right_wheel_pose[0], front_left_wheel_pose[0],
                        ur5e_shoulder_pose[0], ur5e_upper_arm_pose[0], ur5e_forearm_pose[0], ur5e_wrist_1_pose[0], 
                        ur5e_wrist_2_pose[0], ur5e_wrist_3_pose[0]])

        sb_rot = self.r.as_matrix()
        T_sb = np.eye(4)
        T_sb[0,3] = self.x
        T_sb[1,3] = self.y
        T_sb[2,3] = self.z
        T_sb[:3, :3] = sb_rot  # 베이스 프레임 회전 행렬
        T_b0 = np.eye(4)
        T_b0[0,3] = 0.1315 # 0.1015
        T_b0[2,3] = 0.51921  # 0.47921
        T_0e = self.ur5e_robot.fkine(self.q[2:]).A

        T = T_b0 @ T_0e  # 베이스 프레임 기준 end-effector 위치
        H_current = SE3(T)  # 현재 end-effector 위치


        # 로봇의 목표 위치를 사람의 현재 위치로 설정

        # 로봇이 사람을 따라가기
        T_cur = T_sb @ T  # 현재 로봇 위치 (월드 좌표계 기준)
        cur_p = T_cur[:3, 3]  # 현재 엔드 이펙터 위치 (월드 좌표계 기준)

        # 엔드 이펙터의 변환 행렬
        T_e = T_cur  # 월드 좌표계에서 엔드 이펙터 좌표계로의 변환

        # robot_target_position을 엔드 이펙터 좌표계로 변환
        robot_target_position_homogeneous = np.append(self.human_position, 1)  # 동차 좌표로 확장
        robot_target_position_local = np.linalg.inv(T_e) @ robot_target_position_homogeneous
        robot_target_position_local = robot_target_position_local[:3]  # 3차원으로 변환

        # 현재 엔드 이펙터 위치를 엔드 이펙터 좌표계로 변환 (항상 원점)

        # 목표 방향 계산 (엔드 이펙터 좌표계 기준)
        direction_vector = robot_target_position_local # - cur_p_local
        direction_vector /= np.linalg.norm(direction_vector)  # 방향 벡터 정규화

        # 로봇의 현재 x축 방향 (엔드 이펙터의 x축)
        current_x_axis = T_e[:3, 0]  # 엔드 이펙터 변환 행렬의 첫 번째 열

        # 엔드 이펙터 기준의 방향 벡터 (direction_vector)를 월드 좌표계로 변환
        direction_vector_homogeneous = np.append(direction_vector, 0)  # 방향 벡터는 동차 좌표로 확장 (위치가 아니므로 마지막 값은 0)
        direction_vector_world = T_e[:3, :3] @ direction_vector_homogeneous[:3]  # 회전 행렬만 적용하여 월드 좌표계로 변환

        # z_axis를 월드 좌표계 기준으로 설정
        z_axis = direction_vector_world / np.linalg.norm(direction_vector_world)  # 정규화

        # y축은 현재 x축 방향과 z축의 외적
        y_axis = np.cross(current_x_axis, z_axis)
        y_axis /= np.linalg.norm(y_axis)  # 정규화

        # x축은 y축과 z축의 외적
        x_axis = np.cross(z_axis, y_axis)
        x_axis /= np.linalg.norm(x_axis)  # 정규화

        # 회전 행렬 생성
        rotation_matrix = np.vstack([z_axis, y_axis, x_axis]).T

        # 로봇의 목표 위치 설정
        T_sd = np.eye(4)
        T_sd[:3, :3] = rotation_matrix #T_ee[:3,:3] #rotation_matrix # T_er[:3, :3]  # 회전 행렬은 단위 행렬로 설정
        det = np.linalg.det(rotation_matrix)
        orthogonality_check = np.allclose(rotation_matrix.T @ rotation_matrix, np.eye(3))

        if not np.isclose(det, 1.0) or not orthogonality_check:
            print("Invalid rotation matrix detected. Normalizing...")
            U, _, Vt = np.linalg.svd(rotation_matrix)
            rotation_matrix_normalized = U @ Vt
            T_bd[:3, :3] = rotation_matrix_normalized

        T_sd[0, 3] = self.human_position[0] #robot_target_position[0]
        T_sd[1, 3] = self.human_position[1] #robot_target_position[1]
        T_sd[2, 3] = self.human_position[2] #robot_target_position[2]

        
        # points_between에 있는 점들을 월드 좌표계로 변환하고 구를 생성
        points_world = []:
        # for i, point in enumerate(self.points_between):
        #     # 점을 월드 좌표계로 변환
        #     point_homogeneous = np.append(point, 1)  # 동차 좌표로 확장
        #     point_world = T_e @ point_homogeneous  # 엔드 이펙터 좌표계에서 월드 좌표계로 변환
        #     point_world = point_world[:3]  # 3차원으로 변환
        #     points_world.append(point_world)

   
        # End-effector 위치와 points_between의 양 끝 점을 잇는 직선 그리기
        # for i in range(len(points_world)):
        #     start_point = points_world[0]  # points_between의 첫 번째 점
        #     end_point = points_world[-1]  # points_between의 마지막 점
        #     plt.plot([start_point[0], end_point[0]], [start_point[1], end_point[1]], color='purple', linestyle='-', label="End-effector Line")

        points_world = np.array(points_world)  # (num_points, 3) 형태로 변환
        xform_pose = np.vstack((xform_pose, points_world))  # 현재 xform_pose에 점 추가

        T_bd = np.linalg.inv(T_sb) @ T_sd  
        # print("T_bd:", T_bd)
        # print("T_bd shape:", T_bd.shape)
        H_desired = SE3(T_bd)  # 목표 end-effector 위치

        F = np.array([[0.0, 1.0],
                        [0.0, 0.0],
                        [0.0, 0.0],
                        [0.0, 0.0], 
                        [0.0, 0.0],
                        [1.0, 0.0]])

        J_p = base.tr2adjoint(T.T) @ F  # 6x2 자코비안 (선형 속도)
        J_a_e = base.tr2adjoint(T.T) @ self.ur5e_robot.jacob0(self.q[2:])
        J_mb = np.hstack((J_p, J_a_e))  # 6x8 자코비안 (선형 속도 + 각속도)
        J_mb_v = J_mb[:3, :]  # 3x8 자코비안 (선형 속도)
        J_mb_w = J_mb[3:, :]  # 3x8 자코비안 (각속도)

       

        T_error = np.linalg.inv(H_current.A) @ H_desired.A  # 4x4

        et = np.sum(np.abs(T_error[:3, -1])) 

        # Gain term (lambda) for control minimisation
        Y = 100

        # Quadratic component of objective function
        Q = np.eye(self.n_dof + 6)

        # Joint velocity component of Q
        Q[:2, :2] *= 1.0 / (et*100)

        # Slack component of Q
        Q[self.n_dof :, self.n_dof :] = (1.0 / et) * np.eye(6)

        H = np.zeros((self.n_dof-2, 6, self.n_dof-2))  # same as jacobm

        for j in range(self.n_dof-2):
            for i in range(j, self.n_dof-2):
                H[j, :3, i] = np.cross(J_mb_w[:, j], J_mb_v[:, i])
                H[j, 3:, i] = np.cross(J_mb_w[:, j], J_mb_w[:, i])
                if i != j:
                        H[i, :3, j] = H[j, :3, i]
                        H[i, 3:, j] = H[j, 3:, i]

        # manipulability only for arm joints
        J_a = self.ur5e_robot.jacob0(self.q[2:])
        m = J_a @ J_a.T 
        m_det = np.linalg.det(m)  
        m_t = np.sqrt(m_det)  # manipulability (sqrt(det(J * J^T)))

        rank = np.linalg.matrix_rank(J_a @ J_a.T)
        if rank < J_a.shape[0]:
            print("Warning: Jacobian matrix is rank-deficient. Robot may be in a singularity.")
            JJ_inv = np.linalg.pinv(J_a @ J_a.T)  # 유사역행렬 사용
        else:
            JJ_inv = np.linalg.inv(J_a @ J_a.T)  # 역행렬 계산

        # Compute manipulability Jacobian only for arm joints
        J_m = np.zeros((self.n_dof-2,1))
        for i in range(self.n_dof-2):
            c = J_a @ np.transpose(H[i, :, :])  # shape: (6,6)
            J_m[i,0] = m_t * np.transpose(c.flatten("F")) @ JJ_inv.flatten("F")

        A = np.zeros((self.n_dof + 2 + self.num_points, self.n_dof + 6))
        B = np.zeros(self.n_dof + 2 + self.num_points)
        # print(f"Ashape: {A.shape}, B shape: {B.shape}")

        J_dj = np.zeros(self.n_dof+6)
        w_p_sum = 0.0
        min_dist_list = []  # 장애물과의 최소 거리 리스트
        for i , pose in enumerate(xform_pose) :

            distance, index, g_vec = self.get_nearest_obstacle_distance(pose, self.obstacles_positions[:, :3], self.obstacle_radius, T_cur)
            min_dist = np.min(distance)
            min_dist_list.append(min_dist)  # 최소 거리 추가
            # print('min_dist', min_dist)
            
            if i < 4:  # mobile base wheels
            
                position_homogeneous = np.append(pose, 1)  # 동차 좌표로 확장
                position_local = np.linalg.inv(T_e) @ position_homogeneous
                position_local = position_local[:3]  # 3차원으로 변환
                dist_T = np.eye(4)
                dist_T[:3, 3] = position_local
                T_ = T @ dist_T
                J_p_ = base.tr2adjoint(T_.T) @ F  # 6x2 자코비안 (선형 속도)
                J_a_e_ = base.tr2adjoint(T_.T) @ self.ur5e_robot.jacob0(self.q[2:])
                J_mb_ = np.hstack((J_p_, J_a_e_))  # 6x8 자코비안 (선형 속도 + 각속도)
                J_mb_v_ = J_mb_[:3, :]  # 3x8 자코비안 (선형 속도)
                # J_mb_arm_v_ = np.hstack([J_mb_v_, np.zeros((3, 6))])

                d_dot = (-g_vec) @ J_mb_v_ # J_mb_arm_v_
                
                A[i, :8] = d_dot 
                A[i, 8:] = np.zeros((1, 6)) 
                B[i] = (min_dist - self.d_safe) / (self.d_influence - self.d_safe) 
                w_p = (self.d_influence-min_dist)/(self.d_influence-self.d_safe) 
                J_dj[:8] += A[i, :8] * w_p  # 베이스 조인트 속도에 대한 제약 조건
                w_p_sum += w_p
                if min_dist < 0.0:
                    print(f"robot {i+1}th link is too close to an obstacle. Distance: {min_dist:.2f} m")
                    print(f"A : {A[i, :8]}")
                    print(f"B : {B[i]:.2f}")
            elif 3 < i < 10:  # UR5e joints
                
                J_mb_arm_v = np.hstack([np.zeros((3, i - 2)), J_mb_v[:3, i - 2: ]])
                d_dot = (-g_vec) @ J_mb_arm_v

                A[i, :8] = d_dot
                A[i, 8:] = np.zeros((1, 6)) 
                B[i] = (min_dist - self.d_safe) / (self.d_influence - self.d_safe)
                w_p = (self.d_influence-min_dist)/(self.d_influence-self.d_safe) 
                J_dj[:8] += A[i, :8] * w_p  # 베이스 조인트 속도에 대한 제약 조건
                w_p_sum += w_p


            else:
                position_homogeneous = np.append(pose, 1)  # 동차 좌표로 확장
                position_local = np.linalg.inv(T_e) @ position_homogeneous
                position_local = position_local[:3]  # 3차원으로 변환
                dist_T = np.eye(4)
                dist_T[:3, 3] = position_local
                T_ = T @ dist_T
                J_p_ = base.tr2adjoint(T_.T) @ F  # 6x2 자코비안 (선형 속도)
                J_a_e_ = base.tr2adjoint(T_.T) @ self.ur5e_robot.jacob0(self.q[2:])
                J_mb_ = np.hstack((J_p_, J_a_e_))  # 6x8 자코비안 (선형 속도 + 각속도)
                J_mb_v_ = J_mb_[:3, :]  # 3x8 자코비안 (선형 속도)
                J_mb_arm_v_ = np.hstack([np.zeros((3, 7)), J_mb_v_[:3, 7: ]])

                d_dot = (-g_vec) @ J_mb_arm_v_

                A[i, :8] = d_dot
                A[i, 8:] = np.zeros((1, 6)) 
                B[i] = (min_dist - self.d_safe) / (self.d_influence - self.d_safe)
                w_p = (self.d_influence-min_dist)/(self.d_influence-self.d_safe) 

                J_dj[:8] += A[i, :8] * w_p  # 베이스 조인트 속도에 대한 제약 조건
                w_p_sum += w_p

        C = np.concatenate((np.zeros(2), 8.0*J_m.reshape((self.n_dof - 2,)), np.zeros(6)))
        bTe = self.ur5e_robot.fkine(self.q[2:], include_base=False).A  
        θε = atan2(bTe[1, -1], bTe[0, -1])
        # world에서 사람의 좌표 world_human_position에 넣어야함 (3,) vector
        weight_param = np.sum(np.abs(self.human_position - T_e[:3, 3]))

        if weight_param < 0.5:
            k_e = 1.0
        else:
            k_e = 6.0
        C[0] = - k_e * θε  # 베이스 x 위치 오차

        min_distance = np.min(min_dist_list)  # 장애물과의 최소 거리
        lambda_c = (self.lambda_max /(self.d_influence - self.d_safe)**2) * (min_distance - self.d_influence)**2
        J_c = lambda_c * J_dj/w_p_sum

        C += J_c # 베이스 조인트 속도에 대한 제약 조건 추가

        J_ = np.c_[J_mb, np.eye(6)]  # J_ 행렬 (예시)

        eTep = np.linalg.inv(T) @ H_desired.A  # 현재 위치에서의 오차 행렬

        e = np.zeros(6)

        # Translational error
        e[:3] = eTep[:3, -1]

        # Angular error
        e[3:] = base.tr2rpy(eTep, unit="rad", order="zyx", check=False)
        # print(f"e: {e}")
        k = np.eye(6)  # gain
        k[:3,:] *= 8.0 # gain
        v = k @ e
        v[3:] *= 1.3

        lb = -np.r_[self.qdlim[: self.n_dof], 10 * np.ones(6)]
        ub = np.r_[self.qdlim[: self.n_dof], 10 * np.ones(6)]
        # print(f"Qshape: {Q.shape}, C shape: {C.shape}, A shape: {A.shape}, B shape: {B.shape}, J_ shape: {J_.shape}, v shape: {v.shape}, lb shape: {lb.shape}, ub shape: {ub.shape}")
        qd = qp.solve_qp(Q,C,A,B,J_,v,lb=lb, ub=ub, solver='quadprog')

     
        qd = qd[:8]

        if qd is None:
            print("QP solution is None")
            qd = np.array([0.,0.,0.0,0.0,0.,0.,0.,0.]) 

        if et > 0.5:
            qd = qd[: self.n_dof]
            # qd = 2 * qd
            # qd[:2] = 2 * qd[:2] 
        elif 0.5 > et > 0.2 : 
            qd = qd[: self.n_dof]
            # qd = 1.5 * qd
        else:
            qd = qd[: self.n_dof]
            # qd[:2] = 0.5 * qd[:2] 

            # qd[2:] = 0.5 * qd[2:]  # UR5e 조인트 속도 증가
            # qd = 0.5 * qd
            print("et:", et)


        wc, vc = qd[0], qd[1]  # 베이스 속도
        # wc *= 2.0

        twist = Twist()
        twist.linear.x = vc
        twist.angular.z = wc
        self.scout_publisher.publish(twist)
        # joint_vel = JointState()
        # joint_vel.velocity = qd[2:]
        # self.ur5e_publisher.publish(joint_vel)
        self.rtde_c.speedJ(qd[2:], 0.2, self.dt)
        self.rtde_c.waitPeriod(t_start)
        # r_m = 0.165
        # l_m = 0.582
        # w_R = vc/r_m + l_m*wc/(2*r_m)
        # w_L = vc/r_m - l_m*wc/(2*r_m)
        # joint_velocities = np.zeros(16)
        # joint_velocities[0] = -w_L
        # joint_velocities[2] = -w_L
        # joint_velocities[1] = w_R
        # joint_velocities[3] = w_R
        # joint_velocities[4:10] = qd[2:]