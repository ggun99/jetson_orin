import numpy as np
from scipy.spatial.transform import Rotation as R
from math import atan2 as atan2
import numpy as np
import rclpy
from rclpy.node import Node 
from mocap4r2_msgs.msg import RigidBodies
from tf2_ros import StaticTransformBroadcaster
from geometry_msgs.msg import TransformStamped

from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point
from std_msgs.msg import ColorRGBA
from geometry_msgs.msg import PoseStamped
import threading
import sys
import termios
import tty

class QP_mbcontorller(Node):
    def __init__(self):
        super().__init__('mbcontroller')
        self.positions = self.create_subscription(RigidBodies, '/rigid_bodies', self.get_transform, 10)
        self.tf_broadcaster = StaticTransformBroadcaster(self)
        self.lower_marker_publisher = self.create_publisher(Marker, 'lower_visualization_marker', 10)
        self.upper_marker_publisher = self.create_publisher(Marker, 'upper_visualization_marker', 10)

        self.robot_collision_check = []
        self.lower_base_pose_list = []
        self.base_pose_list = []
        # keyboard publisher
        self.pose_pub = self.create_publisher(PoseStamped, "marker_pose", 10)

        # 마커 초기 위치
        self.hand_pose = PoseStamped()
        self.hand_pose_status = False
        self.hand_pose.pose.position.x = 0.0
        self.hand_pose.pose.position.y = 0.0
        self.hand_pose.pose.position.z = 0.0
        self.hand_pose.header.frame_id = "map"
        self.ee_pose = None
        # 키보드 입력을 별도 쓰레드에서 처리
        # thread = threading.Thread(target=self.keyboard_loop(self.ee_pose))
        # thread.daemon = True
        # thread.start()
        thread = threading.Thread(target=self.keyboard_loop)
        thread.daemon = True
        thread.start()

    def get_transform(self, msg):
        
        for i in range(len(msg.rigidbodies)):
            if msg.rigidbodies[i].rigid_body_name == '444':
                base_position = [msg.rigidbodies[i].pose.position.x,
                                msg.rigidbodies[i].pose.position.y,
                                msg.rigidbodies[i].pose.position.z]
                self.ee_pose = base_position
                self.base_pose_list = [
                                        [marker.translation.x, marker.translation.y, marker.translation.z]
                                        for marker in msg.rigidbodies[i].markers
                                    ]
                base_quat = [msg.rigidbodies[i].pose.orientation.x,
                             msg.rigidbodies[i].pose.orientation.y,
                             msg.rigidbodies[i].pose.orientation.z,
                             msg.rigidbodies[i].pose.orientation.w]
                self.make_tf_msg(base_position, base_quat, "base")

            elif msg.rigidbodies[i].rigid_body_name == '111':
                hand_position = [msg.rigidbodies[i].pose.position.x,
                                msg.rigidbodies[i].pose.position.y,
                                msg.rigidbodies[i].pose.position.z]
                hand_quat = [msg.rigidbodies[i].pose.orientation.x,
                             msg.rigidbodies[i].pose.orientation.y,
                             msg.rigidbodies[i].pose.orientation.z,
                             msg.rigidbodies[i].pose.orientation.w]
                self.make_tf_msg(hand_position, hand_quat, "hand")
                
            elif msg.rigidbodies[i].rigid_body_name == '555':
                self.base_position = [msg.rigidbodies[i].pose.position.x,
                                      msg.rigidbodies[i].pose.position.y,
                                      msg.rigidbodies[i].pose.position.z]
                self.robot_collision_check = [
                    (marker.translation.x, marker.translation.y, marker.translation.z)
                    for marker in msg.rigidbodies[i].markers
                ]
                self.lower_base_pose_list = [
                                        [marker.translation.x, marker.translation.y, marker.translation.z]
                                        for marker in msg.rigidbodies[i].markers
                                    ]
        
        self.publish_marker(self.lower_base_pose_list, "lower_base", 1, ColorRGBA(r=0.0, g=1.0, b=0.0, a=1.0), self.lower_marker_publisher)
        
        self.publish_marker(self.base_pose_list, "upper_base", 0, ColorRGBA(r=1.0, g=0.0, b=0.0, a=1.0), self.upper_marker_publisher)

    def make_tf_msg(self, pos, quat, child_frame_name):
        tfmsg = TransformStamped()
        tfmsg.header.stamp = self.get_clock().now().to_msg()
        tfmsg.header.frame_id = "map"
        tfmsg.child_frame_id = child_frame_name
        tfmsg.transform.translation.x = pos[0]
        tfmsg.transform.translation.y = pos[1]
        tfmsg.transform.translation.z = pos[2]
        tfmsg.transform.rotation.x = quat[0]
        tfmsg.transform.rotation.y = quat[1]
        tfmsg.transform.rotation.z = quat[2]
        tfmsg.transform.rotation.w = quat[3]
        self.tf_broadcaster.sendTransform(tfmsg)


    def publish_marker(self, pos_list, marker_name, marker_id, marker_color, publisher):
        marker = Marker()
        marker.header.frame_id = "map"  # RViz에서 맞는 TF frame으로 변경
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = marker_name
        marker.id = marker_id
        marker.type = Marker.POINTS
        marker.action = Marker.ADD

        # scale: POINTS는 x,y 값이 width/height를 의미
        marker.scale.x = 0.1  # 점의 가로 크기
        marker.scale.y = 0.1  # 점의 세로 크기

        # 색상 (전체 공통 색 지정 가능)
        marker.color = marker_color  

        # Points 추가
        points = []
        for i in range(len(pos_list)):
            p = Point()
            p.x = pos_list[i][0]
            p.y = pos_list[i][1]
            p.z = pos_list[i][2]
            points.append(p)

        marker.points = points

        publisher.publish(marker)


    def publish_pose(self):
        self.pose_pub.publish(self.hand_pose)
        self.get_logger().info(
            f"x={self.hand_pose.pose.position.x:.2f}, "
            f"y={self.hand_pose.pose.position.y:.2f}, "
            f"z={self.hand_pose.pose.position.z:.2f}"
        )

    def keyboard_loop(self):
        if self.hand_pose_status is False and self.ee_pose is not None:
            self.hand_pose.pose.position.x = self.ee_pose[0]
            self.hand_pose.pose.position.y = self.ee_pose[1]
            self.hand_pose.pose.position.z = self.ee_pose[2]

        print("Use WASD to move X/Y, QE to move Z. Ctrl+C to quit.")
        print("W: +Y, S: -Y, A: -X, D: +X, Q: +Z, E: -Z")
        settings = termios.tcgetattr(sys.stdin)

        try:
            tty.setcbreak(sys.stdin.fileno())
            while True:
                key = sys.stdin.read(1)
                if key == 'w':
                    self.hand_pose.pose.position.y += 0.1
                elif key == 's':
                    self.hand_pose.pose.position.y -= 0.1
                elif key == 'a':
                    self.hand_pose.pose.position.x -= 0.1
                elif key == 'd':
                    self.hand_pose.pose.position.x += 0.1
                elif key == 'q':
                    self.hand_pose.pose.position.z += 0.1
                elif key == 'e':
                    self.hand_pose.pose.position.z -= 0.1
                elif key == '\x03':  # Ctrl+C
                    break
                else:
                    continue

                self.publish_pose()

        finally:
            termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)

if __name__ == '__main__':
    rclpy.init()
    node = QP_mbcontorller()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
    