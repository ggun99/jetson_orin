#!/usr/bin/env python3
"""
원형 궤적 발행기 (Circular Trajectory Publisher)
기존 직선 궤적 발행기와 유사한 구조로 원형 궤적을 생성하고 발행
"""

import numpy as np
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Pose
from std_msgs.msg import Bool, Int32
import time
import signal
import sys

class CircularTrajectoryPublisher(Node):
    def __init__(self):
        super().__init__('circular_trajectory_publisher')
        
        # Publisher
        self.pose_publisher = self.create_publisher(Pose, '/hand_pose', 10)
        self.trajectory_status_publisher = self.create_publisher(Bool, '/trajectory_completed', 10)
        self.current_waypoint_publisher = self.create_publisher(Int32, '/current_waypoint', 10)
        
        # Subscriber - 외부 트리거 받음
        self.next_waypoint_sub = self.create_subscription(
            Bool, '/next_waypoint_trigger', self.next_waypoint_callback, 10)
        
        # 원형 궤적 매개변수
        self.center_x = 0.3  # 원의 중심 x 좌표 (m)
        self.center_y = 0.0  # 원의 중심 y 좌표 (m)
        self.center_z = 0.2  # 원의 중심 z 좌표 (m)
        self.radius = 0.1    # 원의 반지름 (m) = 10cm
        self.num_points = 16  # 원 위의 점 개수
        self.height = 0.2    # 고정 높이 (m)
        
        # 궤적 생성
        self.generate_circular_trajectory()
        
        # 상태 변수
        self.current_waypoint_idx = 0
        self.trajectory_active = True
        self.waypoint_reached = False
        self.waiting_for_robot = False
        self.trajectory_completed = False
        
        # 시그널 핸들러 설정
        signal.signal(signal.SIGINT, self.signal_handler)
        signal.signal(signal.SIGTERM, self.signal_handler)
        
        print("🔄 원형 궤적 발행 노드 시작")
        print(f"📐 원형 궤적 정보:")
        print(f"   - 중심점: ({self.center_x:.2f}, {self.center_y:.2f}, {self.center_z:.2f}) m")
        print(f"   - 반지름: {self.radius:.2f} m")
        print(f"   - 웨이포인트 개수: {self.num_points}")
        print("📋 로봇이 목표에 도달해야 다음 포인트로 이동합니다")
        
        # 시작 명령 구독 (선택적)
        self.start_sub = self.create_subscription(
            Bool, '/start_trajectory', self.start_trajectory_callback, 10)
        
        # 첫 번째 목표 발행
        self.publish_current_waypoint()
        
        # 타이머 (50Hz) - 현재 목표만 계속 발행
        self.timer = self.create_timer(0.02, self.publish_current_target)
        
        # 즉시 첫 번째 목표 발행
        self.publish_first_target()

    def generate_circular_trajectory(self):
        """원형 궤적 생성"""
        self.waypoints = []
        
        # 원 위의 점들 생성 (시계 반대 방향)
        for i in range(self.num_points):
            angle = 2 * np.pi * i / self.num_points  # 0부터 2π까지
            
            # 원형 궤적 계산
            x = self.center_x + self.radius * np.cos(angle)
            y = self.center_y + self.radius * np.sin(angle)
            z = self.center_z  # 고정 높이
            
            # Pose 메시지 생성
            pose = Pose()
            pose.position.x = x
            pose.position.y = y
            pose.position.z = z
            
            # 기본 방향 (아래를 향함)
            pose.orientation.x = 0.0
            pose.orientation.y = 1.0
            pose.orientation.z = 0.0
            pose.orientation.w = 0.0
            
            self.waypoints.append(pose)
        
        print(f"✅ {len(self.waypoints)}개의 원형 궤적 웨이포인트 생성 완료")
        
        # 생성된 웨이포인트 출력
        for i, wp in enumerate(self.waypoints):
            angle_deg = (360.0 * i / self.num_points)
            print(f"   WP{i+1:2d}: ({wp.position.x:.3f}, {wp.position.y:.3f}, {wp.position.z:.3f}) - {angle_deg:5.1f}°")

    def start_trajectory_callback(self, msg):
        """궤적 시작 명령 콜백"""
        if msg.data:
            print("🚀 궤적 시작 명령 수신")
            self.trajectory_active = True
            self.trajectory_completed = False
            self.current_waypoint_idx = 0
            self.publish_current_waypoint()

    def next_waypoint_callback(self, msg):
        """다음 웨이포인트 트리거 콜백"""
        if msg.data and self.trajectory_active and not self.trajectory_completed:
            self.move_to_next_waypoint()

    def move_to_next_waypoint(self):
        """다음 웨이포인트로 이동"""
        if self.trajectory_completed:
            return
            
        self.current_waypoint_idx += 1
        
        # 한 바퀴 완성 체크
        if self.current_waypoint_idx >= len(self.waypoints):
            print("🎯 원형 궤적 한 바퀴 완성!")
            
            # 원형 궤적은 무한 반복하도록 설정
            self.current_waypoint_idx = 0  # 다시 첫 번째 포인트부터
            print("🔄 원형 궤적 반복 시작...")
            
            # 또는 궤적을 종료하고 싶다면 아래 코드 사용:
            # self.trajectory_completed = True
            # self.publish_trajectory_status()
            # print("✅ 원형 궤적 완료! 프로그램을 종료합니다.")
            # rclpy.shutdown()
            # return
        
        self.publish_current_waypoint()
        print(f"➡️  다음 웨이포인트로 이동: WP{self.current_waypoint_idx + 1}")

    def publish_current_waypoint(self):
        """현재 웨이포인트 정보 발행"""
        if self.trajectory_completed:
            return
            
        current_wp = self.waypoints[self.current_waypoint_idx]
        
        # 현재 웨이포인트 번호 발행
        waypoint_msg = Int32()
        waypoint_msg.data = self.current_waypoint_idx
        self.current_waypoint_publisher.publish(waypoint_msg)
        
        angle_deg = (360.0 * self.current_waypoint_idx / self.num_points)
        print(f"📍 현재 웨이포인트: WP{self.current_waypoint_idx + 1} / {len(self.waypoints)}")
        print(f"   위치: ({current_wp.position.x:.3f}, {current_wp.position.y:.3f}, {current_wp.position.z:.3f})")
        print(f"   각도: {angle_deg:.1f}°")

    def publish_current_target(self):
        """현재 목표 위치 발행 (50Hz)"""
        if self.trajectory_completed:
            return
            
        if len(self.waypoints) > 0 and self.current_waypoint_idx < len(self.waypoints):
            current_target = self.waypoints[self.current_waypoint_idx]
            self.pose_publisher.publish(current_target)

    def publish_first_target(self):
        """첫 번째 목표 즉시 발행"""
        if len(self.waypoints) > 0:
            first_target = self.waypoints[0]
            self.pose_publisher.publish(first_target)
            print("🎯 첫 번째 목표 위치 발행됨")

    def publish_trajectory_status(self):
        """궤적 완료 상태 발행"""
        status_msg = Bool()
        status_msg.data = self.trajectory_completed
        self.trajectory_status_publisher.publish(status_msg)

    def signal_handler(self, signum, frame):
        """시그널 핸들러"""
        print("\n🛑 프로그램 종료 신호 수신")
        self.trajectory_completed = True
        self.publish_trajectory_status()
        rclpy.shutdown()
        sys.exit(0)

def main(args=None):
    """메인 함수"""
    rclpy.init(args=args)
    
    try:
        node = CircularTrajectoryPublisher()
        print("🔄 원형 궤적 발행 노드 실행 중...")
        print("종료하려면 Ctrl+C를 누르세요.")
        
        rclpy.spin(node)
        
    except KeyboardInterrupt:
        print("\n🛑 사용자에 의한 종료")
    except Exception as e:
        print(f"❌ 오류 발생: {e}")
    finally:
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == '__main__':
    main()
