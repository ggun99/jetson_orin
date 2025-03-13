import rclpy
from rclpy.node import Node
from geometry_msgs.msg import WrenchStamped
import time
import matplotlib.pyplot as plt

class GraphNode(Node):
    def __init__(self):
        super().__init__('graph_node')
        self.subscription = self.create_subscription(
            WrenchStamped,
            '/ur_ftsensor',
            self.listener_callback,
            10
        )
        self.time_data = []
        self.force_data_x = []
        self.force_data_y = []
        self.force_data_z = []
        self.max_time = 45
        self.max_data_length = 400  # Maximum number of data points to display
        self.start_time = time.time()

    def listener_callback(self, msg):
        cur_time = time.time() - self.start_time
        print(cur_time)
        # Append new data
        self.time_data.append(cur_time)
        self.force_data_x.append(msg.wrench.force.x)
        self.force_data_y.append(msg.wrench.force.y)
        self.force_data_z.append(msg.wrench.force.z)

        # Keep only the last 500 data points
        # if cur_time < self.max_time: #len(self.time_data) > self.max_data_length:
        #     self.time_data.pop(0)
        #     self.force_data_x.pop(0)
        #     self.force_data_y.pop(0)
        #     self.force_data_z.pop(0)

        # If 500 data points are collected, plot the graph
        if cur_time > self.max_time:  #len(self.time_data) == self.max_data_length:
            self.plot_graph()

    def plot_graph(self):
        plt.figure(figsize=(10, 6))
        plt.plot(self.time_data, self.force_data_x, label='Force X', color='red')
        plt.plot(self.time_data, self.force_data_y, label='Force Y', color='blue')
        plt.plot(self.time_data, self.force_data_z, label='Force Z', color='green')
        plt.xlabel('Time (s)')
        plt.ylabel('Force (N)')
        plt.title('Force Data Over Time')
        plt.legend()
        plt.grid()
        plt.show()


def main():
    rclpy.init()
    graph_node = GraphNode()
    try:
        rclpy.spin(graph_node)
    except KeyboardInterrupt:
        print("Shutting down.")
        rclpy.shutdown()

if __name__ == '__main__':
    import time
    time.sleep(5)
    main()
