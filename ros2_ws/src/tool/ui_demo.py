#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import sys, time, threading
from PyQt5 import QtWidgets, QtCore, QtGui
from PyQt5.QtGui import QPalette, QColor, QFont
from PyQt5.QtGui import QLinearGradient, QRadialGradient, QBrush, QPixmap
from PyQt5.QtWidgets import QGraphicsDropShadowEffect

# ====== UI 參數 ======
CHECKPOINTS = [
    (0, "Start A", "1"),
    (1, "Reset B", "2"),
    (2, "Reset C", "3"),
    (3, "Reset D", "4"),
]
UPDATE_MS = 100

# ====== ROS 嘗試載入（沒有也能跑）======
ROS_OK = True
HAVE_ODOM = False
try:
    import rclpy
    from rclpy.node import Node
    from geometry_msgs.msg import Twist
    from std_msgs.msg import String
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
TOPIC_RESET = "/reset_cmd"

# ====== THEME（顏色集中）======
THEME = {
    "bg": "#1D2632",
    "surface": "#233041",
    "surface2": "#2B3A4F",
    "border": "#3B4A60",
    "text": "#EEF3F9",
    "subtext": "#ABBBCE",
    "accent": "#46A0FF",
    "accent_hover": "#6AB6FF",
    "accent_press": "#2B7EE8",
}

def build_qss(c):
    return f"""
    QWidget {{
        background: palette(Window);
        color: {c['text']};
        font-family: 'Inter', 'Noto Sans TC', 'PingFang TC', 'Microsoft JhengHei', sans-serif;
    }}

    /* 速度框（兩個 QLabel） */
    QLabel#SpeedBox {{
        font-size: 22px;
        font-weight: 600;
        color: {c['text']};
        background: qlineargradient(x1:0, y1:0, x2:0, y2:1,
                    stop:0 {c['surface2']}, stop:1 {c['surface']});
        border: 1px solid {c['border']};
        border-radius: 14px;
        padding: 14px 16px;
        letter-spacing: 0.2px;
    }}

    /* 大按鈕 */
    QPushButton#BigButton {{
        font-size: 40px;
        font-weight: 700;
        background-color: {c['surface']};
        color: {c['text']};
        border: 1px solid {c['border']};
        border-radius: 16px;
        padding: 16px 18px;
    }}
    QPushButton#BigButton:hover {{
        background-color: {c['surface2']};
        border-color: {c['accent']};
    }}
    QPushButton#BigButton:pressed {{
        background-color: {c['accent_press']};
        border-color: {c['accent_press']};
        color: white;
    }}
    QPushButton#BigButton:focus {{
        outline: none;
        border: 2px solid {c['accent_hover']};
    }}
    QPushButton#BigButton:disabled {{
        color: {c['subtext']};
        background-color: {c['surface2']};
        border-color: {c['border']};
    }}
    """

def make_palette(c):
    pal = QPalette()
    pal.setColor(QPalette.Window, QColor(c["bg"]))
    pal.setColor(QPalette.Base, QColor(c["surface"]))
    pal.setColor(QPalette.AlternateBase, QColor(c["surface2"]))
    pal.setColor(QPalette.WindowText, QColor(c["text"]))
    pal.setColor(QPalette.Text, QColor(c["text"]))
    pal.setColor(QPalette.Button, QColor(c["surface"]))
    pal.setColor(QPalette.ButtonText, QColor(c["text"]))
    pal.setColor(QPalette.Highlight, QColor(c["accent"]))
    pal.setColor(QPalette.HighlightedText, QColor("#ffffff"))
    return pal

# ====== ROS 客戶端 ======
class RosClient(Node if ROS_OK else object):
    def __init__(self):
        if ROS_OK:
            super().__init__("ui_panel_speed_viewer")
            self._lock = threading.Lock()
            self._latest = (0.0,0.0,0.0, 0.0,0.0,0.0)
            # 訂閱 /cmd_vel
            self.create_subscription(Twist, TOPIC_TWIST, self._cb_twist, 10)
            # 可選 /odom
            if HAVE_ODOM:
                self.create_subscription(Odometry, TOPIC_ODOM, self._cb_odom, 10)

    def _cb_twist(self, msg: 'Twist'):
        if not ROS_OK: return
        with getattr(self, "_lock", threading.Lock()):
            self._latest = (
                float(msg.linear.x), float(msg.linear.y), float(msg.linear.z),
                float(msg.angular.x), float(msg.angular.y), float(msg.angular.z)
            )

    def _cb_odom(self, msg: 'Odometry'):
        if not ROS_OK: return
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

# ====== 主面板 ======
class Panel(QtWidgets.QWidget):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("Reset Panel")
        root = QtWidgets.QVBoxLayout(self)
        root.setContentsMargins(16, 16, 16, 16)
        root.setSpacing(14)

        # ---- Linear 速度框 ----
        self.linear_box = QtWidgets.QLabel("Linear\nvx=—  vy=—  vz=—")
        self.linear_box.setObjectName("SpeedBox")
        self.linear_box.setAlignment(QtCore.Qt.AlignCenter)
        self.linear_box.setMinimumHeight(84)
        root.addWidget(self.linear_box)

        # ---- Angular 速度框 ----
        self.angular_box = QtWidgets.QLabel("Angular\nwx=—  wy=—  wz=—")
        self.angular_box.setObjectName("SpeedBox")
        self.angular_box.setAlignment(QtCore.Qt.AlignCenter)
        self.angular_box.setMinimumHeight(84)
        root.addWidget(self.angular_box)

        # ---- 2×2 按鈕格 ----
        grid = QtWidgets.QGridLayout()
        grid.setContentsMargins(0, 0, 0, 0)
        grid.setSpacing(14)
        root.addLayout(grid)

        self._hotkey_map = {}
        self._buttons = []
        for i, (cid, cname, hk) in enumerate(CHECKPOINTS):
            btn = QtWidgets.QPushButton(cname)
            btn.setObjectName("BigButton")
            btn.setSizePolicy(QtWidgets.QSizePolicy.Expanding, QtWidgets.QSizePolicy.Expanding)
            btn.setMinimumSize(240, 140)
            btn.clicked.connect(lambda _, a=cid, b=cname, w=btn: self.on_trigger(a, b, w))
            r, c = divmod(i, 2)
            grid.addWidget(btn, r, c)
            self._hotkey_map[hk] = (cid, cname, btn)
            self._buttons.append(btn)

        for k in range(2):
            grid.setColumnStretch(k, 1)
            grid.setRowStretch(k, 1)
        self.resize(900, 600)

        # ====== ROS 啟動 & 定時刷新 ======
        self.ros_node = None
        self.spin_thread = None
        self.pub_reset = None
        if ROS_OK:
            try:
                rclpy.init(args=None)
                self.ros_node = RosClient()
                self.pub_reset = self.ros_node.create_publisher(String, TOPIC_RESET, 10)
                self.spin_thread = RosSpinThread(self.ros_node)
                self.spin_thread.start()
            except Exception:
                # 若 ROS 初始化失敗，仍讓 UI 可用
                self.ros_node = None
                self.pub_reset = None

        self.timer = QtCore.QTimer(self)
        self.timer.timeout.connect(self.refresh_speed)
        self.timer.start(UPDATE_MS)

        # ====== 套用主題 ======
        self.apply_theme()

    # 主題：QSS + Palette + 陰影 + 等寬字體
    def apply_theme(self):
        QtWidgets.QApplication.setStyle("Fusion")
        QtWidgets.QApplication.instance().setPalette(make_palette(THEME))
        self.setStyleSheet(build_qss(THEME))

        # 速度數字用等寬字體
        mono = QFont("JetBrains Mono")
        if not mono.exactMatch():
            mono = QFont("Consolas")
        mono.setPointSize(18)
        self.linear_box.setFont(mono)
        self.angular_box.setFont(mono)

        # 柔和陰影
        def add_shadow(w, blur=28, dx=0, dy=6, alpha=160):
            eff = QGraphicsDropShadowEffect(self)
            eff.setBlurRadius(blur)
            eff.setOffset(dx, dy)
            eff.setColor(QColor(0, 0, 0, alpha))
            w.setGraphicsEffect(eff)

        add_shadow(self.linear_box, blur=30, dy=8, alpha=170)
        add_shadow(self.angular_box, blur=30, dy=8, alpha=170)
        for b in self._buttons:
            add_shadow(b, blur=24, dy=6, alpha=140)
        self.set_background("twilight")   # 可改成 "aurora" / "plum" / "tint" / 'image:/path/to/bg.jpg'
    
    def set_background(self, mode="twilight"):
        """用 palette 設定視窗背景：支援漸層 / 圖片。"""
        self.setAttribute(QtCore.Qt.WA_StyledBackground, True)
        self.setAutoFillBackground(True)
        pal = self.palette()

        if mode == "twilight":  # 深藍靛紫漸層（冷調不死黑）
            g = QLinearGradient(0, 0, 1, 1)
            g.setCoordinateMode(QtGui.QGradient.ObjectBoundingMode)
            g.setColorAt(0.0, QColor("#111726"))
            g.setColorAt(1.0, QColor("#1B2330"))
            pal.setBrush(QPalette.Window, QBrush(g))

        elif mode == "aurora":  # 青綠湖面漸層（深色但有彩度）
            g = QLinearGradient(0, 0, 0, 1)
            g.setCoordinateMode(QtGui.QGradient.ObjectBoundingMode)
            g.setColorAt(0.0, QColor("#0E1E1E"))
            g.setColorAt(1.0, QColor("#112A3A"))
            pal.setBrush(QPalette.Window, QBrush(g))

        elif mode == "plum":  # 暖紫葡萄調（偏創意風）
            g = QRadialGradient(0.3, 0.3, 1.0)
            g.setCoordinateMode(QtGui.QGradient.ObjectBoundingMode)
            g.setColorAt(0.0, QColor("#23182C"))
            g.setColorAt(1.0, QColor("#141120"))
            pal.setBrush(QPalette.Window, QBrush(g))

        elif mode == "tint":  # 單色彩色底（非黑非白）
            pal.setBrush(QPalette.Window, QBrush(QColor("#16222E")))  # 想換色改這行

        elif mode.startswith("image:"):  # 背景圖片（自動鋪滿）
            path = mode.split(":", 1)[1]
            pm = QPixmap(path).scaled(self.size(),
                                    QtCore.Qt.KeepAspectRatioByExpanding,
                                    QtCore.Qt.SmoothTransformation)
            pal.setBrush(QPalette.Window, QBrush(pm))
            self._bg_image_path = path  # 記住路徑，resize 時重貼

        self.setPalette(pal)

    def resizeEvent(self, e):
        super().resizeEvent(e)
        if hasattr(self, "_bg_image_path"):
            pm = QPixmap(self._bg_image_path).scaled(self.size(),
                                                    QtCore.Qt.KeepAspectRatioByExpanding,
                                                    QtCore.Qt.SmoothTransformation)
            pal = self.palette()
            pal.setBrush(QPalette.Window, QBrush(pm))
            self.setPalette(pal)
            ratio = self.devicePixelRatioF() if hasattr(self, "devicePixelRatioF") else 1.0
            pm = QPixmap(self._bg_image_path)
            scaled = pm.scaled(int(self.width()*ratio),
                                int(self.height()*ratio),
                                QtCore.Qt.KeepAspectRatioByExpanding,
                                QtCore.Qt.SmoothTransformation)
            if ratio != 1.0:
                scaled.setDevicePixelRatio(ratio)
            pal = self.palette()
            pal.setBrush(QPalette.Window, QBrush(scaled))
            self.setPalette(pal)

    def refresh_speed(self):
        if self.ros_node is None:
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
        # 短暫高亮
        orig = btn.styleSheet()
        btn.setStyleSheet(orig + f"\nQPushButton#BigButton {{ background-color: {THEME['accent']}; color: white; }}")
        QtCore.QTimer.singleShot(150, lambda: btn.setStyleSheet(orig))

        # 發 ROS 指令
        if self.pub_reset is not None and self.ros_node is not None:
            try:
                msg = String()
                msg.data = f"R{cid+1}"
                self.pub_reset.publish(msg)
                print(f"[UI->ROS] {msg.data}")
            except Exception as e:
                print(f"[UI] 發布失敗: {e}")

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
    app.setPalette(make_palette(THEME))  # 讓對話框等小元件也同步風格
    w = Panel()
    w.show()
    sys.exit(app.exec_())

if __name__ == "__main__":
    main()
