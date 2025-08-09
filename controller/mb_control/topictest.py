

import rclpy
from rclpy.node import Node 
from geometry_msgs.msg import Twist, Pose
from sensor_msgs.msg import JointState
from mocap4r2_msgs.msg import RigidBodies

class QP_mbcontorller(Node):
    def __init__(self):
        super().__init__('mbcontroller')
        self.marker_subscription = self.create_subscription(RigidBodies, '/rigid_bodies', self.marker, 10)
        # self.human_position = self.create_subscription(Pose, '/human_position', self.QP_real, 10)
       

    def marker(self, msg):
        # print(msg.rigidbodies[0].markers[0].translation)
        print(msg.rigidbodies[0].pose)




if __name__ == '__main__':
    rclpy.init()
    mb_controller = QP_mbcontorller()
    rclpy.spin(mb_controller)
    mb_controller.destroy_node()
    rclpy.shutdown()
   