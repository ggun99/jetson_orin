import rclpy
from rclpy.node import Node
from PyQt5 import QtWidgets
import pyqtgraph as pg
import threading
import time
import rtde_receive, rtde_control

class GraphNode(Node):
    def __init__(self):
        super().__init__('graph_node')
        self.ROBOT_IP = '192.168.0.212'  # 로봇의 IP 주소
        self.rtde_r = rtde_receive.RTDEReceiveInterface(self.ROBOT_IP)
        
        self.time_data = []
        self.force_data = [[], [], []]
        self.max_data_length = 500
        self.start_time = time.time()
        self.rtde_c = rtde_control.RTDEControlInterface(self.ROBOT_IP)
        self.rtde_c.zeroFtSensor()

    def start_graph(self):
        app = QtWidgets.QApplication([])

        # PyQtGraph 설정
        win = pg.GraphicsLayoutWidget(show=True, title="Real-Time Plot")
        win.resize(800, 400)
        win.setWindowTitle('Real-Time Plot')

        plot = win.addPlot(title="Real-Time Force Data")
        plot.setLabel('left', 'Force (N)')
        plot.setLabel('bottom', 'Time (s)')
        self.curve1 = plot.plot(pen='r', name="Force X")
        self.curve2 = plot.plot(pen='g', name="Force Y")
        self.curve3 = plot.plot(pen='b', name="Force Z")

        # PyQtGraph 업데이트 타이머
        timer = pg.QtCore.QTimer()
        timer.timeout.connect(self.update_graph)
        timer.start(100)  # 100ms 간격으로 업데이트

        app.exec_()

    def update_graph(self):
        # RTDE 데이터 읽기
        wrench = self.rtde_r.getActualTCPForce()
        if not self.rtde_r.isConnected():
            self.rtde_r.reconnect()
        # wrench = self.rtde_r.getFtRawWrench()

        cur_time = time.time() - self.start_time

        # 데이터 추가 및 크기 제한
        self.time_data.append(cur_time)
        self.force_data[0].append(wrench[0])
        self.force_data[1].append(wrench[1])
        self.force_data[2].append(wrench[2])

        if len(self.time_data) > self.max_data_length:
            self.time_data.pop(0)
            for i in range(3):
                self.force_data[i].pop(0)

        # 그래프 데이터 갱신
        self.curve1.setData(self.time_data, self.force_data[0])
        self.curve2.setData(self.time_data, self.force_data[1])
        self.curve3.setData(self.time_data, self.force_data[2])

def main():
    rclpy.init()
    graph_node = GraphNode()

    # PyQtGraph는 별도의 스레드에서 실행
    graph_thread = threading.Thread(target=graph_node.start_graph, daemon=True)
    graph_thread.start()

    try:
        rclpy.spin(graph_node)  # ROS2 이벤트 루프 실행
    except KeyboardInterrupt:
        print("Shutting down.")
        rclpy.shutdown()

if __name__ == '__main__':
    main()