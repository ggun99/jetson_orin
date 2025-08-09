

import rclpy
from rclpy.node import Node 
from geometry_msgs.msg import Twist, Pose
from sensor_msgs.msg import JointState
from mocap4r2_msgs.msg import RigidBodies

class QP_mbcontorller(Node):
    def __init__(self):
        super().__init__('mbcontroller')
        self.marker_subscription = self.create_subscription(RigidBodies, '/rigid_bodies', self.marker, 10)
    
        # self.positions = self.create_subscription(RigidBodies, '/rigid_bodies', self.set_positions, 10)
        

    def set_positions(self, msg):
        """
        Set the positions of the rigid bodies from the message.
        This function is called when a new message is received on the '/rigid_bodies' topic.
        """
        # 이름 확인해서 넣는걸로 
        for i in range(len(msg.rigidbodies)):
            if msg.rigidbodies[i].rigid_body_name == '111':
                self.human_position = [msg.rigidbodies[i].pose.position.x, 
                                       msg.rigidbodies[i].pose.position.y, 
                                       msg.rigidbodies[i].pose.position.z]
            elif msg.rigidbodies[i].rigid_body_name == '222':
                self.obstacles_positions = [msg.rigidbodies[i].pose.position.x, 
                                       msg.rigidbodies[i].pose.position.y, 
                                       msg.rigidbodies[i].pose.position.z]
            elif msg.rigidbodies[i].rigid_body_name == '333':
                self.points_between = [
                                        (marker.translation.x, marker.translation.y, marker.translation.z)
                                        for marker in msg.rigidbodies[i].markers
                                    ]
            elif msg.rigidbodies[i].rigid_body_name == '444':
                self.base_position = [msg.rigidbodies[i].pose.position.x,
                                      msg.rigidbodies[i].pose.position.y,
                                      msg.rigidbodies[i].pose.position.z]
                self.base_quaternion = [
                    msg.rigidbodies[i].pose.orientation.x,
                    msg.rigidbodies[i].pose.orientation.y,
                    msg.rigidbodies[i].pose.orientation.z,
                    msg.rigidbodies[i].pose.orientation.w
                ]
            elif msg.rigidbodies[i].rigid_body_name == '555':
                self.robot_collision_check = [
                    (marker.translation.x, marker.translation.y, marker.translation.z)
                    for marker in msg.rigidbodies[i].markers
                ]
        print("Human Position:", self.human_position)
        # print("Obstacles Positions:", self.obstacles_positions)
        # print("Points Between:", self.points_between)
        # print("Base Position:", self.base_position)
        # print("Robot Collision Check:", self.robot_collision_check)
# mocap4r2_msgs.msg.RigidBody(rigid_body_name='1', markers=[], 
# pose=geometry_msgs.msg.Pose(position=geometry_msgs.msg.Point(x=1.0854297876358032, y=-0.7204346060752869, z=1.039568543434143), 
# orientation=geometry_msgs.msg.Quaternion(x=-0.7252913117408752, y=-0.525711715221405, z=0.04790082201361656, w=0.44191083312034607)))

    def marker(self, msg):
        # print(msg.rigidbodies[0].markers[0].translation)
        # print(msg.rigidbodies[0].rigid_body_name)
        print(msg.rigidbodies[0])




if __name__ == '__main__':
    rclpy.init()
    mb_controller = QP_mbcontorller()
    rclpy.spin(mb_controller)
    mb_controller.destroy_node()
    rclpy.shutdown()
   