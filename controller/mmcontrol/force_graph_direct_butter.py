import rclpy
from rclpy.node import Node
from PyQt5 import QtWidgets
import pyqtgraph as pg
import threading
import time
import rtde_receive, rtde_control
from scipy.signal import butter, lfilter

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
        self.ft_x = []
        self.ft_y = []
        self.ft_z = []

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

    def butter_lowpass(self, cutoff, fs, order=5):
        nyq = 0.5 * fs
        normal_cutoff = cutoff / nyq
        b, a = butter(order, normal_cutoff, btype='low', analog=False)
        return b, a

    # 필터 적용 함수
    def butter_lowpass_filter(self, data, cutoff, fs, order=5):
        b, a = self.butter_lowpass(cutoff, fs, order=order)
        y = lfilter(b, a, data)
        return y

    def update_graph(self):
        # RTDE 데이터 읽기
        wrench = self.rtde_r.getActualTCPForce()
        if not self.rtde_r.isConnected():
            self.rtde_r.reconnect()
        # wrench = self.rtde_r.getFtRawWrench()
        ft = wrench
        if ft:
            # Filter requirements.
            cutoff = 3.0  # 저역통과 필터의 컷오프 주파수
            fs = 100.0     # 프레임 속도 (초당 프레임)
            order = 3     # 필터 차수
            self.ft_x.append(ft[0])
            self.ft_y.append(ft[1])
            self.ft_z.append(ft[2])
            print('ft_nfilt', ft)
            if len(self.ft_x) > 5 :
                self.ft_x.pop(0)
                self.ft_y.pop(0)
                self.ft_z.pop(0)
            # 데이터가 충분할 때 필터 적용
            
            filtered_ft_x = self.butter_lowpass_filter(self.ft_x, cutoff, fs, order)
            filtered_ft_y = self.butter_lowpass_filter(self.ft_y, cutoff, fs, order)
            filtered_ft_z = self.butter_lowpass_filter(self.ft_z, cutoff, fs, order)
                # ft[0] = filtered_ft_x[-1]
                # ft[1] = filtered_ft_y[-1]
                # ft[2] = filtered_ft_z[-1]
                # print('ft_filtered', ft)

        cur_time = time.time() - self.start_time


        # 데이터 추가 및 크기 제한
        self.time_data.append(cur_time)
        self.force_data[0].append(filtered_ft_x[-1])
        self.force_data[1].append(filtered_ft_y[-1])
        self.force_data[2].append(filtered_ft_z[-1])
        
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