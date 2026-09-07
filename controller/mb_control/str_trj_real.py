#!/usr/bin/env python3
import numpy as np
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Pose
from std_msgs.msg import Bool, Int32
import time

class TrajectoryPublisher(Node):
    def __init__(self):
        super().__init__('trajectory_publisher')
        
        # Publisher
        self.pose_publisher = self.create_publisher(Pose, '/hand_pose', 10)
        self.trajectory_status_publisher = self.create_publisher(Bool, '/trajectory_completed', 10)
        self.current_waypoint_publisher = self.create_publisher(Int32, '/current_waypoint', 10)
        
        # Subscriber - 외부 트리거만 받음
        self.next_waypoint_sub = self.create_subscription(
            Bool, '/next_waypoint_trigger', self.next_waypoint_callback, 10)
        
        # 궤적 설정
        self.define_trajectories()
        
        # 상태 변수
        self.current_trajectory = 0
        self.trajectory_active = True  # 항상 활성화 (목표 위치만 계속 발행)
        self.waypoint_reached = False
        self.waiting_for_robot = False  # 로봇이 도달하기를 기다리는 상태
        self.trajectory_completed = False  # 전체 궤적 완료 상태
        
        print("🚀 x축 20cm 단위 궤적 발행 노드 시작 (외부 트리거 전용)")
        print("📋 로봇이 목표에 도달해야 다음 포인트로 이동합니다")
        
        # 시작 명령 구독 (선택적)
        self.start_sub = self.create_subscription(
            Bool, '/start_trajectory', self.start_trajectory_callback, 10)
        
        # 첫 번째 목표 발행
        print("📍 첫 번째 웨이포인트 발행 준비...")
        self.publish_current_waypoint()
        
        # 타이머 (50Hz) - 현재 목표만 계속 발행
        self.timer = self.create_timer(0.02, self.publish_current_target)
        
        # 즉시 첫 번째 목표 발행 (디버깅용)
        self.publish_first_target()

    def define_trajectories(self):
        """20cm 단위 직선 궤적들 정의"""
        
        # x축을 따라 이동하는 20cm 단위 직선 궤적들
        base_y = -0.  # y 좌표 고정
        base_z = 0.9  # z 좌표 고정
        
        # x축 범위: 0.5 → 2.5 (총 2.0m를 20cm 단위로 분할)
        x_positions = np.arange(2.0, 2.6, 0.05)  # [0.5, 0.7, 0.9, ..., 2.5]
        
        self.trajectories = []
        
        # 앞으로 가는 궤적들 (0.5 → 2.5)
        for i in range(len(x_positions) - 1):
            waypoint = {
                'name': f'X_Forward_{i+1}',
                'position': np.array([x_positions[i+1], base_y, base_z]),  # 목표 지점만 저장
                'orientation': [0.0, 0.0, 0.0, 1.0]  # [x, y, z, w]
            }
            self.trajectories.append(waypoint)
        
        # 뒤로 가는 궤적들 (2.5 → 0.5) - 주석 해제
        # for i in range(len(x_positions) - 1):
        #     waypoint = {
        #         'name': f'X_Backward_{i+1}',
        #         'position': np.array([x_positions[-(i+1)], base_y, base_z]),  # 역순
        #         'orientation': [0.0, 0.0, 0.0, 1.0]
        #     }
        #     self.trajectories.append(waypoint)
        
        print(f"📈 {len(self.trajectories)}개의 웨이포인트 정의 완료")
        for i, waypoint in enumerate(self.trajectories):
            print(f"   {i:2d}: {waypoint['name']:15s} → x: {waypoint['position'][0]:.1f}")

    def publish_first_target(self):
        """첫 번째 목표를 즉시 발행 (초기화용)"""
        if len(self.trajectories) > 0:
            first_waypoint = self.trajectories[0]
            
            # Pose 메시지 생성
            pose_msg = Pose()
            pose_msg.position.x = float(first_waypoint['position'][0])
            pose_msg.position.y = float(first_waypoint['position'][1])
            pose_msg.position.z = float(first_waypoint['position'][2])
            
            pose_msg.orientation.x = float(first_waypoint['orientation'][0])
            pose_msg.orientation.y = float(first_waypoint['orientation'][1])
            pose_msg.orientation.z = float(first_waypoint['orientation'][2])
            pose_msg.orientation.w = float(first_waypoint['orientation'][3])
            
            # 발행
            self.pose_publisher.publish(pose_msg)
            
            print(f"🎯 첫 번째 목표 발행 완료!")
            print(f"   웨이포인트 0: {first_waypoint['name']}")
            print(f"   목표 위치: ({first_waypoint['position'][0]:.3f}, {first_waypoint['position'][1]:.3f}, {first_waypoint['position'][2]:.3f})")

    def start_trajectory_callback(self, msg):
        """궤적 시작/정지 콜백"""
        if msg.data:
            self.trajectory_active = True
            self.waiting_for_robot = False
            print("🚀 궤적 활성화")
            self.publish_current_waypoint()
        else:
            self.trajectory_active = False
            print("⏹️ 궤적 비활성화")

    def next_waypoint_callback(self, msg):
        """외부 트리거로 다음 웨이포인트로 이동 (핵심 기능)"""
        if msg.data and self.trajectory_active:
            print("📡 로봇 도달 신호 수신! 다음 웨이포인트로 이동")
            self.move_to_next_waypoint()

    def move_to_next_waypoint(self):
        """다음 웨이포인트로 이동"""
        prev_trajectory = self.current_trajectory
        
        if len(self.trajectories) > 0:
            prev_waypoint = self.trajectories[prev_trajectory]
            print(f"✅ 웨이포인트 {prev_trajectory} 완료: {prev_waypoint['name']}")
            
            # 마지막 웨이포인트 확인
            if self.current_trajectory >= len(self.trajectories) - 1:
                print("🏁 모든 웨이포인트 완료! 프로그램을 종료합니다.")
                self.trajectory_completed = True
                self.trajectory_active = False
                
                # 최종 완료 상태 발행
                completion_msg = Bool()
                completion_msg.data = True
                self.trajectory_status_publisher.publish(completion_msg)
                
                # 프로그램 종료
                import threading
                def shutdown_delayed():
                    time.sleep(2.0)  # 2초 후 종료
                    rclpy.shutdown()
                
                shutdown_thread = threading.Thread(target=shutdown_delayed)
                shutdown_thread.start()
                return
            
            # 다음 웨이포인트로 이동
            self.current_trajectory += 1
            current_waypoint = self.trajectories[self.current_trajectory]
            
            print(f"➡️ 다음 웨이포인트: {self.current_trajectory} - {current_waypoint['name']}")
            print(f"   새 목표: x: {current_waypoint['position'][0]:.1f}")
            
            # 새 웨이포인트 정보 발행
            self.publish_current_waypoint()
            
            # 완료 상태 발행
            completion_msg = Bool()
            completion_msg.data = True
            self.trajectory_status_publisher.publish(completion_msg)
            
            self.waiting_for_robot = True  # 다시 로봇 도달 대기

    def publish_current_waypoint(self):
        """현재 웨이포인트 번호 발행"""
        waypoint_msg = Int32()
        waypoint_msg.data = self.current_trajectory
        self.current_waypoint_publisher.publish(waypoint_msg)
        print(f"📤 웨이포인트 ID 발행: {self.current_trajectory}")

    def publish_current_target(self):
        """현재 목표 위치를 계속 발행"""
        if not self.trajectory_active or len(self.trajectories) == 0 or self.trajectory_completed:
            return
        
        current_waypoint = self.trajectories[self.current_trajectory]
        
        # Pose 메시지 생성
        pose_msg = Pose()
        pose_msg.position.x = float(current_waypoint['position'][0])
        pose_msg.position.y = float(current_waypoint['position'][1])
        pose_msg.position.z = float(current_waypoint['position'][2])
        
        # Orientation 설정
        pose_msg.orientation.x = float(current_waypoint['orientation'][0])
        pose_msg.orientation.y = float(current_waypoint['orientation'][1])
        pose_msg.orientation.z = float(current_waypoint['orientation'][2])
        pose_msg.orientation.w = float(current_waypoint['orientation'][3])
        
        # 발행
        self.pose_publisher.publish(pose_msg)
        
        # 상태 표시 (5초마다)
        if hasattr(self, 'last_status_time'):
            if time.time() - self.last_status_time > 5.0:  # 5초마다
                status = "로봇 도달 대기 중..." if self.waiting_for_robot else "목표 발행 중..."
                print(f"📍 웨이포인트 {self.current_trajectory}: {current_waypoint['name']} | {status}")
                print(f"   목표: ({current_waypoint['position'][0]:.1f}, {current_waypoint['position'][1]:.1f}, {current_waypoint['position'][2]:.1f})")
                self.last_status_time = time.time()
        else:
            self.last_status_time = time.time()

    def get_trajectory_info(self):
        """현재 웨이포인트 정보 반환"""
        if self.current_trajectory < len(self.trajectories):
            return self.trajectories[self.current_trajectory]
        return None

    def print_status(self):
        """상태 정보 출력"""
        current_waypoint = self.get_trajectory_info()
        if current_waypoint:
            print(f"\n📊 현재 상태:")
            print(f"   웨이포인트 ID: {self.current_trajectory}/{len(self.trajectories)-1}")
            print(f"   웨이포인트 이름: {current_waypoint['name']}")
            print(f"   목표 위치: ({current_waypoint['position'][0]:.1f}, {current_waypoint['position'][1]:.1f}, {current_waypoint['position'][2]:.1f})")
            print(f"   활성 상태: {self.trajectory_active}")
            print(f"   로봇 대기 상태: {self.waiting_for_robot}")

def main(args=None):
    print("🌟 외부 트리거 기반 웨이포인트 발행 프로그램 시작")
    print("🎯 로봇이 실제로 도달해야 다음 포인트로 이동합니다")
    
    rclpy.init(args=args)
    node = TrajectoryPublisher()
    
    # 상태 정보 출력 타이머
    def print_status():
        if hasattr(node, 'trajectory_active'):
            node.print_status()
    
    # 15초마다 상태 출력
    status_timer = node.create_timer(15.0, print_status)
    
    try:
        print("🔄 ROS2 스핀 시작 (Ctrl+C로 종료)")
        print("⏳ 로봇이 목표에 도달하면 자동으로 다음 포인트로 이동합니다")
        rclpy.spin(node)
    except KeyboardInterrupt:
        print("\n🛑 사용자 종료")
    finally:
        node.destroy_node()
        rclpy.shutdown()
        print("🏁 프로그램 종료")

if __name__ == '__main__':
    main()