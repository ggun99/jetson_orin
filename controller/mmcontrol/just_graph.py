import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32
from geometry_msgs.msg import WrenchStamped
import pyqtgraph as pg
import PySide2
from PySide2 import QtWidgets, QtCore
import time
import threading

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
        self.force_data = [[], [], []]
        self.max_data_length = 500  # Maximum number of data points to display
        self.start_time = time.time()

    def listener_callback(self, msg):
        cur_time = time.time() - self.start_time
        if len(self.time_data) >= self.max_data_length:
            # Remove the oldest data to maintain the size
            self.time_data.pop(0)
            for i in range(3):
                self.force_data[i].pop(0)
        
        # Append new data
        self.time_data.append(cur_time)
        self.force_data[0].append(msg.wrench.force.x)
        self.force_data[1].append(msg.wrench.force.y)
        self.force_data[2].append(msg.wrench.force.z)

    def start_graph(self):
        app = QtWidgets.QApplication([])
        win = pg.GraphicsLayoutWidget(show=True, title="Real-Time Plot")
        win.resize(800, 400)
        win.setWindowTitle('Real-Time Plot')

        plot = win.addPlot(title="Real-Time Data")
        plot.setLabel('left', 'Force')
        plot.setLabel('bottom', 'Time', 's')
        curve1 = plot.plot(pen='r', name="Force X")
        curve2 = plot.plot(pen='g', name="Force Y")
        curve3 = plot.plot(pen='b', name="Force Z")

        while True:
            if self.time_data:  # Update only if there is data
                curve1.setData(self.time_data, self.force_data[0])
                curve2.setData(self.time_data, self.force_data[1])
                curve3.setData(self.time_data, self.force_data[2])
            app.processEvents()
            time.sleep(0.1)  # Adjust as needed for update frequency

def main():
    rclpy.init()
    graph_node = GraphNode()

    # Start graph in a separate thread
    graph_thread = threading.Thread(target=graph_node.start_graph, daemon=True)
    graph_thread.start()

    try:
        rclpy.spin(graph_node)
    except KeyboardInterrupt:
        print("Shutting down.")
        rclpy.shutdown()

if __name__ == '__main__':
    main()