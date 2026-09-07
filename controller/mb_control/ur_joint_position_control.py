#!/usr/bin/env python3
"""
UR 로봇 조인트 위치 제어 스크립트
특정 조인트 값으로 UR 로봇을 이동시키는 프로그램
"""

import rtde_control
import rtde_receive
import numpy as np
import time
import sys

class URJointPositionController:
    def __init__(self, robot_ip='192.160.0.4'):
        """
        UR 로봇 조인트 위치 제어기 초기화
        
        Args:
            robot_ip (str): UR 로봇의 IP 주소
        """
        self.robot_ip = robot_ip
        
        try:
            # RTDE Control Interface 초기화
            self.rtde_c = rtde_control.RTDEControlInterface(self.robot_ip)
            # RTDE Receive Interface 초기화
            self.rtde_r = rtde_receive.RTDEReceiveInterface(self.robot_ip)
            print(f"✅ UR 로봇 연결 성공 (IP: {self.robot_ip})")
        except Exception as e:
            print(f"❌ UR 로봇 연결 실패: {e}")
            sys.exit(1)
    
    def get_current_joint_positions(self):
        """현재 조인트 위치 반환"""
        return self.rtde_r.getActualQ()
    
    def move_to_joint_positions(self, joint_positions, velocity=0.5, acceleration=0.3, asynchronous=False):
        """
        지정된 조인트 위치로 이동
        
        Args:
            joint_positions (list): 6개 조인트 위치 (라디안)
            velocity (float): 최대 조인트 속도 (라디안/초)
            acceleration (float): 최대 조인트 가속도 (라디안/초²)
            asynchronous (bool): 비동기 실행 여부
        """
        if len(joint_positions) != 6:
            print("❌ 조인트 위치는 6개 값이어야 합니다.")
            return False
        
        # 조인트 위치를 라디안으로 변환 (이미 라디안이면 그대로)
        joint_positions_rad = np.array(joint_positions)
        
        print(f"🎯 목표 조인트 위치 (라디안):")
        for i, pos in enumerate(joint_positions_rad):
            print(f"   J{i+1}: {pos:.3f} rad ({np.degrees(pos):.1f}°)")
        
        try:
            # 조인트 이동 실행
            self.rtde_c.moveJ(joint_positions_rad.tolist(), velocity, acceleration, asynchronous)
            
            if not asynchronous:
                print("✅ 조인트 이동 완료")
            else:
                print("🚀 비동기 조인트 이동 시작")
            
            return True
            
        except Exception as e:
            print(f"❌ 조인트 이동 실패: {e}")
            return False
    
    def move_to_degrees(self, joint_degrees, velocity=0.5, acceleration=0.3, asynchronous=False):
        """
        지정된 조인트 위치로 이동 (도 단위)
        
        Args:
            joint_degrees (list): 6개 조인트 위치 (도)
            velocity (float): 최대 조인트 속도 (라디안/초)
            acceleration (float): 최대 조인트 가속도 (라디안/초²)
            asynchronous (bool): 비동기 실행 여부
        """
        joint_radians = [np.radians(deg) for deg in joint_degrees]
        return self.move_to_joint_positions(joint_radians, velocity, acceleration, asynchronous)
    
    def move_to_home_position(self):
        """홈 포지션으로 이동"""
        home_position = [0.0, -np.pi/2, np.pi/2, np.pi, -np.pi/2, 0.0]  # 기본 홈 포지션
        print("🏠 홈 포지션으로 이동 중...")
        return self.move_to_joint_positions(home_position)
    
    def move_to_safe_position(self):
        """안전한 위치로 이동"""
        safe_position = [0.0, -np.pi/3, -np.pi/3, -np.pi/3, np.pi/2, 0.0]
        print("🛡️ 안전 위치로 이동 중...")
        return self.move_to_joint_positions(safe_position)
    
    def print_current_status(self):
        """현재 로봇 상태 출력"""
        current_q = self.get_current_joint_positions()
        print("\n📊 현재 로봇 상태:")
        print(f"   조인트 위치 (라디안): {[f'{q:.3f}' for q in current_q]}")
        print(f"   조인트 위치 (도): {[f'{np.degrees(q):.1f}°' for q in current_q]}")
        
        # TCP 위치 정보
        tcp_pose = self.rtde_r.getActualTCPPose()
        print(f"   TCP 위치 (m): [{tcp_pose[0]:.3f}, {tcp_pose[1]:.3f}, {tcp_pose[2]:.3f}]")
        print(f"   TCP 회전 (rad): [{tcp_pose[3]:.3f}, {tcp_pose[4]:.3f}, {tcp_pose[5]:.3f}]")
    
    def wait_for_movement_completion(self):
        """로봇 움직임이 완료될 때까지 대기"""
        print("⏳ 로봇 움직임 완료 대기 중...")
        while True:
            robot_mode = self.rtde_r.getRobotMode()
            safety_mode = self.rtde_r.getSafetyMode()
            
            # 로봇이 실행 모드이고 안전 모드가 정상일 때
            if robot_mode == 7 and safety_mode == 1:  # RUNNING and NORMAL
                joint_speeds = self.rtde_r.getActualQd()
                # 모든 조인트 속도가 거의 0에 가까우면 정지 상태
                if all(abs(speed) < 0.01 for speed in joint_speeds):
                    break
            
            time.sleep(0.1)
        
        print("✅ 로봇 움직임 완료")
    
    def interactive_control(self):
        """대화형 조인트 제어 모드"""
        print("\n🎮 대화형 UR 조인트 제어 모드")
        print("명령어:")
        print("  'status' - 현재 상태 확인")
        print("  'home' - 홈 포지션으로 이동")
        print("  'safe' - 안전 위치로 이동")
        print("  'move <j1> <j2> <j3> <j4> <j5> <j6>' - 조인트 위치 이동 (도 단위)")
        print("  'mover <j1> <j2> <j3> <j4> <j5> <j6>' - 조인트 위치 이동 (라디안 단위)")
        print("  'quit' - 종료")
        
        while True:
            try:
                command = input("\n명령 입력: ").strip().lower()
                
                if command == 'quit':
                    break
                elif command == 'status':
                    self.print_current_status()
                elif command == 'home':
                    self.move_to_home_position()
                    self.wait_for_movement_completion()
                elif command == 'safe':
                    self.move_to_safe_position()
                    self.wait_for_movement_completion()
                elif command.startswith('move '):
                    try:
                        parts = command.split()
                        if len(parts) != 7:
                            print("❌ 6개의 조인트 값을 입력하세요.")
                            continue
                        
                        joint_degrees = [float(parts[i]) for i in range(1, 7)]
                        self.move_to_degrees(joint_degrees)
                        self.wait_for_movement_completion()
                    except ValueError:
                        print("❌ 잘못된 숫자 형식입니다.")
                elif command.startswith('mover '):
                    try:
                        parts = command.split()
                        if len(parts) != 7:
                            print("❌ 6개의 조인트 값을 입력하세요.")
                            continue
                        
                        joint_radians = [float(parts[i]) for i in range(1, 7)]
                        self.move_to_joint_positions(joint_radians)
                        self.wait_for_movement_completion()
                    except ValueError:
                        print("❌ 잘못된 숫자 형식입니다.")
                else:
                    print("❌ 알 수 없는 명령입니다.")
                    
            except KeyboardInterrupt:
                break
            except Exception as e:
                print(f"❌ 오류 발생: {e}")
    
    def disconnect(self):
        """연결 해제"""
        try:
            self.rtde_c.disconnect()
            self.rtde_r.disconnect()
            print("🔌 UR 로봇 연결 해제 완료")
        except:
            pass

def main():
    """메인 함수"""
    print("🤖 UR 로봇 조인트 위치 제어 프로그램")
    print("="*50)
    
    # 로봇 제어기 초기화
    controller = URJointPositionController()
    
    try:
        # 현재 상태 출력
        controller.print_current_status()
        
        # 미리 정의된 위치들
        predefined_positions = {
            'home': [0, -90, 90, 180, -90, 0],  # 도 단위
            'safe': [0, -60, -60, -60, 90, 0],
            'observe': [0, -45, -90, -45, 90, 0],
            'pickup': [0, -30, -120, -30, 90, 0]
        }
        
        print("\n📋 미리 정의된 위치:")
        for name, pos in predefined_positions.items():
            print(f"   {name}: {pos} (도)")
        
        # 사용자 선택
        print("\n🎯 실행 모드 선택:")
        print("1. 대화형 제어 모드")
        print("2. 미리 정의된 위치로 이동")
        print("3. 현재 상태만 확인")
        
        choice = input("선택 (1-3): ").strip()
        
        if choice == '1':
            controller.interactive_control()
        elif choice == '2':
            print("\n미리 정의된 위치 선택:")
            for i, (name, pos) in enumerate(predefined_positions.items(), 1):
                print(f"{i}. {name}: {pos}")
            
            try:
                pos_choice = int(input("위치 선택 (1-4): ").strip()) - 1
                pos_names = list(predefined_positions.keys())
                
                if 0 <= pos_choice < len(pos_names):
                    selected_name = pos_names[pos_choice]
                    selected_pos = predefined_positions[selected_name]
                    
                    print(f"🎯 '{selected_name}' 위치로 이동 중...")
                    controller.move_to_degrees(selected_pos)
                    controller.wait_for_movement_completion()
                    controller.print_current_status()
                else:
                    print("❌ 잘못된 선택입니다.")
                    
            except ValueError:
                print("❌ 잘못된 입력입니다.")
        elif choice == '3':
            controller.print_current_status()
        else:
            print("❌ 잘못된 선택입니다.")
            
    except KeyboardInterrupt:
        print("\n🛑 프로그램 중단됨")
    except Exception as e:
        print(f"❌ 오류 발생: {e}")
    finally:
        controller.disconnect()

if __name__ == '__main__':
    main()
