#2006wu_08_14

#!/usr/bin/env python3
import sys, time, threading
from PyQt5 import QtWidgets, QtCore

# ====== UI 參數 ======
CHECKPOINTS = [
    (0, "Start A", "1"),
    (1, "Reset B", "2"),
    (2, "Reset C", "3"),
    (3, "Reset D", "4"),
]
SPEED_FMT = "vx={:.2f}  vy={:.2f}  vz={:.2f}   wx={:.2f}  wy={:.2f}  wz={:.2f}"
UPDATE_MS = 100

# ====== ROS 嘗試載入（沒有也能跑）======
ROS_OK = True
try:
    import rclpy
    from rclpy.node import Node
    from geometry_msgs.msg import Twist
    try:
        from nav_msgs.msg import Odometry
        HAVE_ODOM = True
    except Exception:
        HAVE_ODOM = False
except Exception:
    ROS_OK = False
    HAVE_ODOM = False

TOPIC_TWIST = "/cmd_vel"
TOPIC_ODOM  = "/odom"

class RosClient(Node if ROS_OK else object):
    def __init__(self):
        if ROS_OK:
            super().__init__("ui_panel_speed_viewer")
            self._lock = threading.Lock()
            self._latest = (0.0,0.0,0.0,0.0,0.0,0.0)
            # 訂閱 /cmd_vel (Twist)
            self.create_subscription(Twist, TOPIC_TWIST, self._cb_twist, 10)
            # 同時訂閱 /odom（若有）
            if HAVE_ODOM:
                self.create_subscription(Odometry, TOPIC_ODOM, self._cb_odom, 10)

    def _cb_twist(self, msg: 'Twist'):
        with getattr(self, "_lock", threading.Lock()):
            self._latest = (
                float(msg.linear.x), float(msg.linear.y), float(msg.linear.z),
                float(msg.angular.x), float(msg.angular.y), float(msg.angular.z)
            )

    def _cb_odom(self, msg: 'Odometry'):
        t = msg.twist.twist
        with getattr(self, "_lock", threading.Lock()):
            self._latest = (
                float(t.linear.x), float(t.linear.y), float(t.linear.z),
                float(t.angular.x), float(t.angular.y), float(t.angular.z)
            )

    def get_latest(self):
        if not ROS_OK:
            return None
        with self._lock:
            return self._latest

class RosSpinThread(QtCore.QThread):
    def __init__(self, node: RosClient):
        super().__init__()
        self.node = node
        self._running = True
    def run(self):
        if not ROS_OK:
            return
        while self._running and rclpy.ok():
            rclpy.spin_once(self.node, timeout_sec=0.05)
            time.sleep(0.01)
    def stop(self):
        self._running = False

class Panel(QtWidgets.QWidget):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("Reset Panel (Demo)")
        root = QtWidgets.QVBoxLayout(self)
        root.setContentsMargins(12, 12, 12, 12)
        root.setSpacing(12)

        # ---- Linear 速度框 ----
        self.linear_box = QtWidgets.QLabel("Linear\nvx=—  vy=—  vz=—")
        self.linear_box.setAlignment(QtCore.Qt.AlignCenter)
        self.linear_box.setStyleSheet("""
            QLabel {
                font-size: 22px;
                color: white;
                background-color: #333333;
                font-weight: bold;
                border: 1px solid #666666;
                border-radius: 10px;
                padding: 12px;
            }
        """)
        self.linear_box.setMinimumHeight(80)
        root.addWidget(self.linear_box)

        # ---- Angular 速度框 ----
        self.angular_box = QtWidgets.QLabel("Angular\nwx=—  wy=—  wz=—")
        self.angular_box.setAlignment(QtCore.Qt.AlignCenter)
        self.angular_box.setStyleSheet("""
            QLabel {
                font-size: 22px;
                color: white;
                background-color: #333333;
                font-weight: bold;
                border: 1px solid #666666;
                border-radius: 10px;
                padding: 12px;
            }
        """)
        self.angular_box.setMinimumHeight(80)
        root.addWidget(self.angular_box)


        # ---- 2×2 按鈕格 ----
        grid = QtWidgets.QGridLayout()
        grid.setContentsMargins(0, 0, 0, 0)
        grid.setSpacing(12)
        root.addLayout(grid)

        self._hotkey_map = {}
        for i, (cid, cname, hk) in enumerate(CHECKPOINTS):
            btn = QtWidgets.QPushButton(cname)
            btn.setSizePolicy(QtWidgets.QSizePolicy.Expanding, QtWidgets.QSizePolicy.Expanding)
            btn.setMinimumSize(240, 140)
            btn.setStyleSheet("""
            QPushButton {
                font-size: 40px;
                color: white;
                background-color: #333333;
                font-weight: bold;
                border: 1px solid #666666;
                border-radius: 10px;
                padding: 12px;
            }
            QPushButton:pressed { background-color: #444444; }
            """)
            btn.clicked.connect(lambda _, a=cid, b=cname, w=btn: self.on_trigger(a, b, w))
            r, c = divmod(i, 2)
            grid.addWidget(btn, r, c)
            self._hotkey_map[hk] = (cid, cname, btn)

        for k in range(2):
            grid.setColumnStretch(k, 1)
            grid.setRowStretch(k, 1)
        self.resize(900, 600)

        # ---- ROS 啟動 & 定時刷新速度列 ----
        self.ros_node = None
        self.spin_thread = None
        if ROS_OK:
            try:
                rclpy.init(args=None)
                self.ros_node = RosClient()
                self.spin_thread = RosSpinThread(self.ros_node)
                self.spin_thread.start()
            except Exception as e:
                # 若 ROS 初始化失敗，仍讓 UI 可用
                self.ros_node = None

        self.timer = QtCore.QTimer(self)
        self.timer.timeout.connect(self.refresh_speed)
        self.timer.start(UPDATE_MS)

    def refresh_speed(self):
        if self.ros_node is None:
            # 沒連 ROS → 用破折號
            self.linear_box.setText("Linear\nvx=—  vy=—  vz=—")
            self.angular_box.setText("Angular\nwx=—  wy=—  wz=—")
            return
        data = self.ros_node.get_latest()
        if not data:
            self.linear_box.setText("Linear\nvx=—  vy=—  vz=—")
            self.angular_box.setText("Angular\nwx=—  wy=—  wz=—")
            return
        vx, vy, vz, wx, wy, wz = data
        self.linear_box.setText(f"Linear\nvx={vx:.2f}  vy={vy:.2f}  vz={vz:.2f}")
        self.angular_box.setText(f"Angular\nwx={wx:.2f}  wy={wy:.2f}  wz={wz:.2f}")

    def keyPressEvent(self, e):
        t = e.text()
        if t in self._hotkey_map:
            cid, cname, btn = self._hotkey_map[t]
            self.on_trigger(cid, cname, btn)

    def on_trigger(self, cid, cname, btn):
        # 點擊高亮一下
        orig = btn.styleSheet()
        btn.setStyleSheet(orig + "\nQPushButton { background-color: #2d6cdf; color: white; }")
        QtCore.QTimer.singleShot(150, lambda: btn.setStyleSheet(orig))
        # 之後要接發 ROS 指令可在這裡加

    def closeEvent(self, e):
        try:
            if self.spin_thread:
                self.spin_thread.stop()
                self.spin_thread.wait(500)
            if ROS_OK and rclpy.ok():
                if self.ros_node:
                    self.ros_node.destroy_node()
                rclpy.shutdown()
        except Exception:
            pass
        e.accept()

def main():
    app = QtWidgets.QApplication(sys.argv)
    w = Panel()
    w.show()
    sys.exit(app.exec_())

if __name__ == "__main__":
    main()
