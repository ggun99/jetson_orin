import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped

class KeyboardGoalPublisher(Node):
    def __init__(self):
        super().__init__('keyboard_goal_publisher')
        self.pub = self.create_publisher(PoseStamped, 'goal_pose', 10)
        self.pose = PoseStamped()
        self.pose.header.frame_id = "map"
        self.pose.pose.position.x = 0.0
        self.pose.pose.position.y = 0.0
        self.pose.pose.position.z = 0.0
        self.initialized = False
        self.create_subscription(PoseStamped, 'initial_goal_pose', self.init_pose_callback, 1)

    def init_pose_callback(self, msg):
        if not self.initialized:
            self.pose = msg
            self.initialized = True
            print(f"초기 목표값을 받았습니다: {self.pose.pose.position}")

    def run(self):
        while rclpy.ok():
            if not self.initialized:
                rclpy.spin_once(self, timeout_sec=0.1)
                continue
            key = input("WASD/QE로 이동 w: +y, s: -y, a: -x, d: +x, q: +z, e: -z, 엔터로 퍼블리시: ")
            if key == 'w':
                self.pose.pose.position.y += 0.1
            elif key == 's':
                self.pose.pose.position.y -= 0.1
            elif key == 'a':
                self.pose.pose.position.x -= 0.1
            elif key == 'd':
                self.pose.pose.position.x += 0.1
            elif key == 'q':
                self.pose.pose.position.z += 0.1
            elif key == 'e':
                self.pose.pose.position.z -= 0.1
            self.pub.publish(self.pose)
            print(f"Published: {self.pose.pose.position}")

def main():
    rclpy.init()
    node = KeyboardGoalPublisher()
    try:
        node.run()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()