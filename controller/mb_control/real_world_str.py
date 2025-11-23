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
import cvxpy as cp

import rclpy
from rclpy.node import Node 
from geometry_msgs.msg import Twist, Pose, PoseArray
from sensor_msgs.msg import JointState


from tf2_ros import StaticTransformBroadcaster
from geometry_msgs.msg import TransformStamped

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
        self.base_position = self.create_subscription(Pose, '/mobile_base/pose', self.set_base_position, 10)
        self.cable_position = self.create_subscription(PoseArray, '/cable_points', self.set_cable_positions, 10)
        self.human_position = self.create_subscription(Pose, '/hand_pose', self.set_human_position, 10)
        self.eta = 1
        self.qdlim = np.array([0.3]*8)
        self.qdlim[:1] = 0.1  # 베이스 조인트 속도 제한
        self.qdlim[1] = 0.1
        self.qlim = np.array([[-np.inf, -np.inf, -3.14159265, -3.14159265, -3.14159265, -3.14159265, -3.14159265, -3.14159265],
                               [ np.inf, np.inf, 3.14159265,  3.14159265,  3.14159265,  3.14159265,  3.14159265,  3.14159265]])
        self.H_desired = None
        # collision avoidance parameters
        self.d_safe = 0.2
        self.d_influence = 2.0
        self.current_joint_positions = None
        self.q = None
        self.num_points = 10
        self.obstacle_radius = 0.25
        self.lambda_max = 0.32
        self.dt = 0.05
        self.create_timer(0.05, self.QP_real)  # 20Hz
        self.scout_publisher = self.create_publisher(Twist, '/cmd_vel', 10)
        # self.ur5e_publisher = self.create_publisher(JointState, 'ur5e_vel', 10)
        self.tf_broadcaster = StaticTransformBroadcaster(self)
        self.human_position = None
        self.obstacles_positions= None
        self.points_between= None
        self.base_quaternion= None
        self.robot_collision_check= []
        self.lambda_h_a_param = 0.5
        self.w1 = 1.0
        self.w2 = 0.5
        self.w3 = 0.0
        self.w4 = 0.5
        # 로봇 링크별 충돌 검사 점들 정의
        self.define_robot_collision_points()

    def define_robot_collision_points(self):
        """각 링크별 충돌 검사용 점들 정의 (링크 로컬 좌표계 기준)"""
        
        # UR5e 각 링크의 충돌 검사 점들 (링크 프레임 기준)
        self.ur5e_link_points = {
            'base': np.array([
                [0.0, 0.0, 0.0],      # 베이스 중심
                [0.08, 0.08, 0.05],   # 베이스 모서리들
                [-0.08, 0.08, 0.05],
                [0.08, -0.08, 0.05],
                [-0.08, -0.08, 0.05],
                [0.06, 0.0, 0.1],     # 베이스 상단
                [-0.06, 0.0, 0.1],
                [0.0, 0.06, 0.1],
                [0.0, -0.06, 0.1]
            ]),
            'shoulder': np.array([
                [0.0, 0.0, 0.0],
                [0.06, 0.06, 0.08],
                [-0.06, 0.06, 0.08],
                [0.06, -0.06, 0.08],
                [-0.06, -0.06, 0.08],
                [0.0, 0.0, 0.12]
            ]),
            'upper_arm': np.array([
                [0.0, 0.0, 0.0],
                [0.05, 0.0, 0.1],     # 상완 중간 지점들
                [-0.05, 0.0, 0.1],
                [0.0, 0.05, 0.1],
                [0.0, -0.05, 0.1],
                [0.04, 0.0, 0.2],     # 상완 끝 부분
                [-0.04, 0.0, 0.2],
                [0.0, 0.04, 0.25],
                [0.0, -0.04, 0.25]
            ]),
            'forearm': np.array([
                [0.0, 0.0, 0.0],
                [0.04, 0.0, 0.08],    # 전완 중간
                [-0.04, 0.0, 0.08],
                [0.0, 0.04, 0.08],
                [0.0, -0.04, 0.08],
                [0.03, 0.0, 0.15],    # 전완 끝
                [-0.03, 0.0, 0.15],
                [0.0, 0.03, 0.17],
                [0.0, -0.03, 0.17]
            ]),
            'wrist_1': np.array([
                [0.0, 0.0, 0.0],
                [0.03, 0.03, 0.0],
                [-0.03, 0.03, 0.0],
                [0.03, -0.03, 0.0],
                [-0.03, -0.03, 0.0]
            ]),
            'wrist_2': np.array([
                [0.0, 0.0, 0.0],
                [0.03, 0.0, 0.03],
                [-0.03, 0.0, 0.03],
                [0.0, 0.03, 0.03],
                [0.0, -0.03, 0.03]
            ]),
            'wrist_3': np.array([
                [0.0, 0.0, 0.0],
                [0.025, 0.025, 0.02],
                [-0.025, 0.025, 0.02],
                [0.025, -0.025, 0.02],
                [-0.025, -0.025, 0.02]
            ])
        }
        
        # 모바일 베이스 충돌 점들 (베이스 프레임 기준)
        self.mobile_base_points = np.array([
            [0.35, 0.25, 0.1],    # 베이스 모서리들 (Scout 크기 고려)
            [-0.35, 0.25, 0.1],
            [0.35, -0.25, 0.1],
            [-0.35, -0.25, 0.1],
            [0.3, 0.2, 0.3],      # 베이스 상단
            [-0.3, 0.2, 0.3],
            [0.3, -0.2, 0.3],
            [-0.3, -0.2, 0.3]
            # [0.25, 0.0, 0.4],     # 베이스 중앙 상단
            # [-0.25, 0.0, 0.4],
            # [0.0, 0.2, 0.4],
            # [0.0, -0.2, 0.4]
        ])

    def compute_dynamic_robot_collision_points(self):
        """현재 로봇 상태에서 모든 충돌 검사 점들의 월드 좌표 계산"""
        
        if self.base_position is None or self.base_quaternion is None:
            return []
        
        collision_points_world = []
        
        # 1. 모바일 베이스 변환 행렬
        T_sb = np.eye(4)
        T_sb[0,3] = self.base_position[0]
        T_sb[1,3] = self.base_position[1] 
        T_sb[2,3] = self.base_position[2] 
        T_sb[:3, :3] = R.from_quat(self.base_quaternion).as_matrix()
        
        # 2. 베이스에서 UR5e 베이스로의 변환
        T_b0 = np.eye(4)
        T_b0[0,3] = 0.1315
        T_b0[2,3] = 0.51921
        
        # 3. 모바일 베이스 점들을 월드 좌표계로 변환
        for point in self.mobile_base_points:
            point_homogeneous = np.append(point, 1)
            point_world = T_sb @ point_homogeneous
            collision_points_world.append(point_world[:3])
        num_mobile = self.mobile_base_points[0]
        # 4. UR5e 각 링크의 점들 변환
        link_names = ['base', 'shoulder', 'upper_arm', 'forearm', 'wrist_1', 'wrist_2', 'wrist_3']
        
        for i, link_name in enumerate(link_names):
            # 각 링크까지의 변환 행렬 계산
            if i == 0:  # base_link
                T_0i = np.eye(4)
            else:  # 다른 링크들
                T_0i = self.ur5e_robot.fkine(self.q[2:2+i]).A
            
            # 월드에서 i번째 링크로의 변환
            T_si = T_sb @ T_b0 @ T_0i
            
            # 해당 링크의 충돌 점들을 월드 좌표계로 변환

            if link_name in self.ur5e_link_points:
                num_mani = 0
                for point in self.ur5e_link_points[link_name]:
                    point_homogeneous = np.append(point, 1)
                    point_world = T_si @ point_homogeneous
                    collision_points_world.append(point_world[:3])
                    num_mani += 1

        return np.array(collision_points_world), num_mobile, num_mani

    def set_base_position(self, msg):
        self.base_position = [msg.pose.position.x,
                            msg.pose.position.y,
                            msg.pose.position.z]
        self.base_quaternion = [
                    msg.pose.orientation.x,
                    msg.pose.orientation.y,
                    msg.pose.orientation.z,
                    msg.pose.orientation.w
                ]
        
    def set_human_position(self, msg):
        self.human_position = [msg.position.x, 
                               msg.position.y, 
                               msg.position.z]
        
    def set_cable_positions(self, msg):
        self.cable_points = [
            (pose.position.x, pose.position.y, pose.position.z)
            for pose in msg.poses
        ]
        # self.points_between
    # def set_positions(self, msg):
    #     """
    #     Set the positions of the rigid bodies from the message.
    #     This function is called when a new message is received on the '/rigid_bodies' topic.
    #     """
    #     # 이름 확인해서 넣는걸로 
        
    #     self.obstacles_positions = [pose.position.x, 
    #                             pose.position.y, 
    #                             pose.position.z]
    #     self.points_between = [
    #                             (marker.translation.x, marker.translation.y, marker.translation.z)
    #                             for marker in markers
    #                         ]
    
    
    #     self.robot_collision_check = [
    #         (marker.translation.x, marker.translation.y, marker.translation.z)
    #         for marker in markers
    #     ]

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
        # T_e = T_cur @ T  # 월드 좌표계에서 엔ee드 이펙터 좌표계로의 변환
        obstacles_local = []
        for obs in obstacles:
            obs_copy = obs.copy() if isinstance(obs, list) else list(obs)  # 복사본 생성
            obs_copy[2] = position[2]
            obs_homogeneous = np.append(obs_copy, 1)  # 동차 좌표로 확장
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

    def make_tf_msg(self, pos, quat, parent_name, child_frame_name):
        tfmsg = TransformStamped()
        tfmsg.header.stamp = self.get_clock().now().to_msg()
        tfmsg.header.frame_id = parent_name
        tfmsg.child_frame_id = child_frame_name
        tfmsg.transform.translation.x = pos[0]
        tfmsg.transform.translation.y = pos[1]
        tfmsg.transform.translation.z = pos[2]
        tfmsg.transform.rotation.x = quat[0]
        tfmsg.transform.rotation.y = quat[1]
        tfmsg.transform.rotation.z = quat[2]
        tfmsg.transform.rotation.w = quat[3]
        self.tf_broadcaster.sendTransform(tfmsg)

    # 여기서 ros를 사용한 것으로 변경
    def QP_real(self):
        t_start = self.rtde_c.initPeriod()
        # sub the joints values
        current_joint_positions = self.rtde_r.getActualQ() # 실제 현재 joint 위치
        # 현재 로봇 베이스의 쿼터니언 회전값
        
        self.q = np.zeros(8)
        self.q[0] = 0.0
        self.q[1] = 0.0 
        self.q[2:] = current_joint_positions  # UR5e 조인트 위치

        # 동적으로 로봇 충돌 점들 계산
        self.robot_collision_check, num_mobile, num_mani = self.compute_dynamic_robot_collision_points()
        xform_pose = list(self.robot_collision_check)

        num_cable = len(self.cable_points)
        xform_pose.append(self.cable_points)


        T_sb = np.eye(4)
        T_sb[0,3] = self.base_position[0]
        T_sb[1,3] = self.base_position[1] 
        T_sb[2,3] = self.base_position[2] 
        T_sb[:3, :3] = R.from_quat(self.base_quaternion).as_matrix() 
        T_b0 = np.eye(4)
        T_b0[0,3] = 0.1315 # 0.1015
        T_b0[2,3] = 0.51921  # 0.47921

        # self.make_tf_msg(fakep, fakeq, "base", "ee_base_all0")
        # print(T_be)
        pppp = list(T_sb[0:3,3])
        # print(pppp)
        qqqq = R.from_matrix(T_sb[0:3,0:3]).as_quat()
        # print(qqqq)
        self.make_tf_msg(pppp, qqqq, "map", "base_world")

        ppose = list(T_b0[0:3,3])
        # print(pppp)
        qqua = R.from_matrix(T_b0[0:3,0:3]).as_quat()
        # print(qqqq)
        self.make_tf_msg(ppose, qqua, "base_world", "base_0")

        # rot_mat = np.eye(4)
        # rot_mat[0,0] = -1.
        # rot_mat[1,1] = -1.
        # rot_mat[2,2] = 1.
        # print(self.q[2:])
        T_0e = self.ur5e_robot.fkine(self.q[2:]).A 
        # print(T_0e)
        # ppppose = list(T_0e[0:3,3])
        # # print(pppp)
        # qqqqua = R.from_matrix(T_0e[0:3,0:3]).as_quat()
        # # print(qqqq)
        # self.make_tf_msg(ppppose, qqqqua, "base_0", "ee_0")

        T = T_b0 @ T_0e  # 베이스 프레임 기준 end-effector 위치

        # correction = np.array([[ 0,  0,  1,  0],
        #                     [-1,  0,  0,  0],
        #                     [ 0, -1,  0,  0],
        #                     [ 0,  0,  0,  1]])

        T_be = T #@ correction
        # print(T_be)
        pppose = list(T_be[0:3,3])
        # print(pppp)
        qqqua = R.from_matrix(T_be[0:3,0:3]).as_quat()
        # print(qqqq)
        self.make_tf_msg(pppose, qqqua, "base_world", "ee_base")

        H_current = SE3(T_be)  # 현재 end-effector 위치
        

        # 각 조인트의 변환 행렬 계산
        # for i in range(1, 7):  # UR5e의 6개의 조인트
        #     T_bi = self.ur5e_robot.fkine(self.q[2:i+2]).A  # 베이스 좌표계에서 i번째 조인트까지의 변환 행렬
        #     T_wi = T_sb @ T_bi  # 월드 좌표계에서 i번째 조인트까지의 변환 행렬
        #     joint_position = T_wi[:3, 3]  # 동차 좌표에서 [x, y, z] 추출
        #     xform_pose.append(joint_position)

        xform_pose = np.array(xform_pose) if len(xform_pose) > 0 else np.array([]).reshape(0, 3)

        # 로봇이 사람을 따라가기
        T_cur = T_sb @ T_be  # 현재 로봇 위치 (월드 좌표계 기준)

        # print(T_cur)
        ppp = list(T_cur[0:3,3])
        # print(ppp)
        qqq = R.from_matrix(T_cur[0:3,0:3]).as_quat()
        # print(qqq)
        self.make_tf_msg(ppp, qqq, "map", "ee")



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
        ppppp = list(T_sd[0:3,3])
        # print(ppppp)
        qqqqq = R.from_matrix(T_sd[0:3,0:3]).as_quat()
        # print(qqqqq)
        self.make_tf_msg(ppppp, qqqqq, "map", "desired")

        # 각도 계산
        sight_vec = T_e[:3,0]
        sight_vec /= np.linalg.norm(sight_vec)
        direction_unit_vector = self.human_position - T_cur[:3, 3]
        direction_unit_vector = direction_unit_vector / np.linalg.norm(direction_unit_vector)
        cos_theta = np.dot(direction_unit_vector, sight_vec)
        theta = np.arccos(np.clip(cos_theta, -1.0, 1.0))
        # theta_values.append(np.degrees(theta))

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

        J_p = base.tr2adjoint(T_be.T) @ F  # 6x2 자코비안 (선형 속도)
        J_a_e = base.tr2adjoint(T_be.T) @ self.ur5e_robot.jacob0(self.q[2:])
        J_mb = np.hstack((J_p, J_a_e))  # 6x8 자코비안 (선형 속도 + 각속도)
        J_mb_v = J_mb[:3, :]  # 3x8 자코비안 (선형 속도)
        J_mb_w = J_mb[3:, :]  # 3x8 자코비안 (각속도)

       

        T_error = np.linalg.inv(H_current.A) @ H_desired.A  # 4x4
        # print(T_error)
        et = np.sum(np.abs(T_error[:3, -1])) 
        # Quadratic component of objective function
        Q = np.eye(self.n_dof + 6)
        # Joint velocity component of Q
        Q[:2, :2] *= 1.0 / max(et * 100, 1e-6)

        # Slack component of Q
        Q[self.n_dof:, self.n_dof:] = (1. / max(et, 1e-6)) * np.eye(6)

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

            distance, index, g_vec = self.get_nearest_obstacle_distance(pose, [self.obstacles_positions], self.obstacle_radius, T_cur)
            min_dist = np.min(distance)
            min_dist_list.append(min_dist)  # 최소 거리 추가
            # print('min_dist', min_dist)
            
            if i < num_mobile:  # mobile base wheels
            
                position_homogeneous = np.append(pose, 1)  # 동차 좌표로 확장
                position_local = np.linalg.inv(T_e) @ position_homogeneous
                position_local = position_local[:3]  # 3차원으로 변환
                dist_T = np.eye(4)
                dist_T[:3, 3] = position_local

                d_dot = (g_vec) @ J_mb_v # J_mb_arm_v_
                
                A[i, :8] = -d_dot 
                A[i, 8:] = np.zeros((1, 6)) 
                B[i] = (min_dist_list[i] - self.d_safe) / (self.d_influence - self.d_safe) 
                w_p = (self.d_influence-min_dist_list[i])/(self.d_influence - self.d_safe) 
                J_dj[:8] += (-d_dot) * w_p  # 베이스 조인트 속도에 대한 제약 조건
                w_p_sum += np.abs(w_p)

                    
            else:  # UR5e joints + cable points
                
                J_mb_arm_v = np.hstack([np.zeros((3, i - num_mobile + 2)), J_a_e[:3, i - num_mobile + 2: ]])
                d_dot = (g_vec) @ J_mb_arm_v

                A[i, :8] = -d_dot
                A[i, 8:] = np.zeros((1, 6)) 
                B[i] = (min_dist_list[i] - self.d_safe) / (self.d_influence - self.d_safe)
                w_p = (self.d_influence-min_dist_list[i])/(self.d_influence - self.d_safe) 
                J_dj[:8] += (-d_dot) * (w_p)  #  #  # 베이스 조인트 속도에 대한 제약 조건
                w_p_sum += w_p


        C1 = np.concatenate((np.zeros(2), -J_m.reshape((self.n_dof - 2,)), np.zeros(6)))
        bTe = self.ur5e_robot.fkine(self.q[2:], include_base=False).A 
        θε = atan2(bTe[1, -1], bTe[0, -1])
        # world에서 사람의 좌표 world_human_position에 넣어야함 (3,) vector
        weight_param = np.sum(np.abs(self.human_position - T_e[:3, 3]))

        if weight_param < 0.5:
            k_e = 1.0
        else:
            k_e = 6.0

        C2 = np.zeros(self.n_dof + 6)
        C2[0] = - k_e * θε  # 베이스 x 위치 오차

        # 장애물 회피 (간단화)
        C3 = np.zeros(self.n_dof + 6)
        min_distance = np.min(min_dist_list)  # 장애물과의 최소 거리
        if min_distance <= self.d_influence :
            lambda_c = (self.lambda_max /(self.d_influence - self.d_safe)**2) * (min_distance - self.d_influence)**2
        else:
            lambda_c = 0.0
        J_c = lambda_c * J_dj/w_p_sum
        C3 = J_c
        
        # 회전 제어 항
        J_h = np.zeros(self.n_dof + 6)
        J_mb_w_h = direction_unit_vector @ J_mb_w

        epsilon = 1e-6
        lambda_h = self.lambda_h_a_param * max(abs(theta), epsilon)
        J_h[:8] = lambda_h * J_mb_w_h

        C4 = J_h
        C =  self.w1 * C1 + self.w2 * C2 + self.w3 * C3 + self.w4 * C4   # 베이스 조인트 속도에 대한 제약 조건 추가

        J_ = np.c_[J_mb, np.eye(6)]  # J_ 행렬 (예시)

        eTep = T_error  # 현재 위치에서의 오차 행렬

        e = np.zeros(6)

        # Translational error
        e[:3] = eTep[:3, -1]

        # Angular error
        e[3:] = base.tr2rpy(eTep, unit="rad", order="zyx", check=False)
        print(f"e: {e}")
        k = np.eye(6)  # gain
        # k[:3,:] *= 8.0 # gain
        v = k @ e
        # v[3:] *= 1.3

        lb = -np.r_[self.qdlim[: self.n_dof], 10 * np.ones(6)]
        ub = np.r_[self.qdlim[: self.n_dof], 10 * np.ones(6)]
        # print(f"Qshape: {Q.shape}, C shape: {C.shape}, A shape: {A.shape}, B shape: {B.shape}, J_ shape: {J_.shape}, v shape: {v.shape}, lb shape: {lb.shape}, ub shape: {ub.shape}")
        # qd = qp.solve_qp(Q,C,A,B,J_,v,lb=lb, ub=ub, solver='quadprog')

     
        
        # qd = [vc, wc, qd1, qd2, qd3, qd4, qd5, qd6]
        # qd = qd[:8]
        # print(f"qd: {qd}")

        # if qd is None:
        #     print("QP solution is None")
        #     qd = np.array([0.,0.,0.0,0.0,0.,0.,0.,0.]) 


        x_ = cp.Variable(self.n_dof+6)
        objective = cp.Minimize(0.5 * cp.quad_form(x_, Q) + C.T @ x_)
        constraints = [
            x_ >= lb,
            x_ <= ub,
            J_ @ x_ == v,
        ]

        prob = cp.Problem(objective, constraints)
        prob.solve(solver=cp.ECOS, verbose=False)

        if x_.value is not None:
            qd = x_.value
        else:
            qd = np.zeros(self.n_dof+6)

        if et > 0.5:
            qd = qd[: self.n_dof]
            
        elif et> 0.1:
            qd = qd[: self.n_dof]
            qd = 0 * qd

        wc, vc = qd[0], qd[1]  # 베이스 속도
        qdc = qd[2:]
        
        # moving base
        twist = Twist()
        twist.linear.x = vc
        twist.angular.z = wc
        # self.scout_publisher.publish(twist)

        # moving arm
        # self.rtde_c.speedJ(qdc, 0.2, self.dt)
        # self.rtde_c.waitPeriod(t_start)

        # joint_vel = JointState()
        # joint_vel.velocity = qd[2:]
        # self.ur5e_publisher.publish(joint_vel)

if __name__ == '__main__':
    rclpy.init()
    node = QP_mbcontorller()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()