# wcz_0819

#!/usr/bin/env python3
import rclpy, threading, re
from rclpy.node import Node
from geometry_msgs.msg import Twist, PointStamped
from std_msgs.msg import Int32, Float64
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy
import serial

# FLOAT_RE = re.compile(r'[-+]?\d+(?:\.\d+)?')
# 數字樣式（整數/浮點皆可）
_FLOAT = r'[-+]?\d+(?:\.\d+)?'
_INT   = r'[-+]?\d+'

class Bridge(Node):
    def __init__(self):
        super().__init__('cmd_vel_serial_bridge')

        # ---------- Parameters ----------
        self.declare_parameter('port', '/dev/ttyACM0')
        self.declare_parameter('baud', 115200)

        # prefix
        self.declare_parameter('arm_point_topic', 'point_out')  # 你的相機/演算法輸出 PointStamped 的 topic
        self.declare_parameter('arm_prefix', 'P')              # 送給 STM 的手臂命令前綴，例如: "P x y z\n"
        self.declare_parameter('vel_prefix', 'V')              # 送給 STM 的底盤命令前綴，例如: "V vx vy wz\n"
        self.declare_parameter('mode_prefix', 'M') 
                     # 送給 STM 的模式命令前綴，例如: "MA mode\n"

        # different mode and three parameters
        self.declare_parameter('mode_topic', 'mode')
        self.declare_parameter('height_topic', 'height')
        self.declare_parameter('angle1_topic', 'angle1')
        self.declare_parameter('angle2_topic', 'angle2')

        port = self.get_parameter('port').value
        baud = int(self.get_parameter('baud').value)

        # self._last_mode = None   # 記住最近一次的模式

        # ---------- Serial ----------
        try:
            self.ser = serial.Serial(
                port, baud, timeout=0.05, write_timeout=0.2,
                rtscts=False, dsrdtr=False
            )
        except Exception as e:
            raise SystemExit(f"Open serial failed: {e}")
        self.get_logger().info(f'Opened {port} @ {baud}')

        # ---------- ROS I/O ----------
        # (A) 底盤速度：保留你原本的 /cmd_vel 行為
        self.create_subscription(Twist, '/cmd_vel', self.cb_cmd, 10)

        # (B) 手臂目標：PointStamped，只取 x/y/z，不用 header
        self.create_subscription(
            PointStamped,
            self.get_parameter('arm_point_topic').value,
            self.cb_pointstamped,
            10
        )

        self.create_subscription(
            Int32,
            "vision_result",
            self.cb_vision_result,
            10
        )

        # (C) 從 STM 回讀的速度 (若 STM 有回三個浮點數，仍轉成 Twist 發到 /stm_vel)
        self.pub_vel = self.create_publisher(Twist, '/stm_vel', 10)

        # (D) two different situation topic: mode / height / angle1 / angle2
        # self.pub_mode   = self.create_publisher(Int32, self.get_parameter('mode_topic').value, 10)
        self.pub_height = self.create_publisher(Float64, self.get_parameter('height_topic').value, 10)
        self.pub_a1     = self.create_publisher(Float64, self.get_parameter('angle1_topic').value, 10)
        self.pub_a2     = self.create_publisher(Float64, self.get_parameter('angle2_topic').value, 10)

        # --- QoS：低延遲、只留最新一筆 ---
        qos_mode = QoSProfile(
            depth=5,
            reliability=QoSReliabilityPolicy.RELIABLE,
            history=QoSHistoryPolicy.KEEP_LAST
        )
        self.pub_mode = self.create_publisher(
            Int32, self.get_parameter('mode_topic').value, qos_mode
        )

        self.pub_mode.publish(Int32(data=0))
        self._last_mode = 0

        # --- 模式定時重發（提升頻率）---
        self.declare_parameter('mode_pub_hz', 2.0)   # 想更快就調高，例如 100.0
        # self._last_mode = None                        # 記住最近一次的模式
        _mode_period = 1.0 / float(self.get_parameter('mode_pub_hz').value)

        # ---------- Reader thread ----------
        self._rx_buf = bytearray()
        self._rx_lock = threading.Lock()
        self._stop = False
        self._reader = threading.Thread(target=self._read_loop, daemon=True)
        self._reader.start()

        self._last_cmd = Twist()       # 最近收到的原始指令（未轉換）

        # --- 可選：固定頻率送（若你不想每次 /cmd_vel 都即時送） ---
        # self._tx_timer = self.create_timer(0.05, self.send_last_cmd)  # 20 Hz
        # self._last_cmd = Twist()  # 初始化

        # change the frequency of sending the info
        # self.timer = self.create_timer(0.05, self.send_last_cmd)  # 20 Hz

        # ---------- Filters & state for RX ACK ----------
        self._last_tx_v = (0.0, 0.0, 0.0)   # 最近一次送出的 (vx, vy, wz)
        self.send_cmdvel(0.0, 0.0, 0.0)
        self._ack_eps   = 0.5               # RX 要接近 TX 的允許誤差（依單位調）
        self.declare_parameter('vx_abs_max', 50.0)
        self.declare_parameter('vy_abs_max', 50.0)
        self.declare_parameter('wz_abs_max', 10.0)
        self._vx_abs_max = float(self.get_parameter('vx_abs_max').value)
        self._vy_abs_max = float(self.get_parameter('vy_abs_max').value)
        self._wz_abs_max = float(self.get_parameter('wz_abs_max').value)

        # 嚴格 ACK 格式
        self._ok_line_re = re.compile(
            rf'^OK\s+({_FLOAT})\s+({_FLOAT})\s+({_FLOAT})\s*$'
        )
        self._pa_line_re = re.compile(
            rf'^PA\s+({_FLOAT})\s+({_FLOAT})\s+({_FLOAT})\s*$'
        )

        self._kv_height_re = re.compile(rf'^\s*height[:\s]+({_FLOAT})\s*$', re.IGNORECASE)
        self._kv_angle1_re = re.compile(rf'^\s*angle1[:\s]+({_FLOAT})\s*$', re.IGNORECASE)
        self._kv_angle2_re = re.compile(rf'^\s*angle2[:\s]+({_FLOAT})\s*$', re.IGNORECASE)

        # === 兩種情境的解析規則 ===
        # 1) MODE：只要是「單一整數」，或 "mode 1" / "MODE:2" 之類
        # 單一整數先不採用
        self._mode_only_int_re = re.compile(r'^\s*([123])\s*$')
        self._mode_keyval_re = re.compile(rf'^\s*mode[:\s]+({_INT})\s*$', re.IGNORECASE)

        # 2) 三參數：height / angle1 / angle2 —— 接受空格或冒號分隔
        self._kv_height_re = re.compile(rf'^\s*height[:\s]+({_FLOAT})\s*$', re.IGNORECASE)
        self._kv_angle1_re = re.compile(rf'^\s*angle1[:\s]+({_FLOAT})\s*$', re.IGNORECASE)
        self._kv_angle2_re = re.compile(rf'^\s*angle2[:\s]+({_FLOAT})\s*$', re.IGNORECASE)

        # # 連續一致性：連兩筆 ACK 近似才發佈（進一步抗雜訊）
        self._ack_prev = None

        # --- 初始化 mode publisher 狀態 ---
        self._current_mode = 0
        self.pub_mode.publish(Int32(data=0))   # 程式啟動就先發 0
        self.get_logger().info("[MODE INIT] 0")

        # 每秒檢查一次，如果還是 0，就再發一次 0
        self._mode_keepalive_timer = self.create_timer(1.0, self._keep_mode_alive)

    # 如果最近一次的 mode 是 0，就持續發 0
    def _keep_mode_alive(self):
        if self._current_mode == 0:
            self.pub_mode.publish(Int32(data=0))

    # --- 你的實際「轉換」函式（依你的情境改寫；若無轉換就直接回傳） ---
    def apply_conversion(self, vx: float, vy: float, wz: float):
        # 例：單位從 m/s 轉 mm/s（示意）
        # scale = 1000.0
        # return vx * scale, vy * scale, wz
        return vx, vy, wz  # 沒轉換就原樣回傳

    # ===================== A) /cmd_vel → 底盤（速度） =====================
    def cb_cmd(self, msg: Twist):
        vx_raw, vy_raw, wz_raw = msg.linear.x, msg.linear.y, msg.angular.z

        # 限幅檢查
        if abs(vx_raw) > 50 or abs(vy_raw) > 50 or abs(wz_raw) > 50:
            self.get_logger().warn(
                f"[LIMIT] Reject cmd: vx={vx_raw:.3f}, vy={vy_raw:.3f}, wz={wz_raw:.3f}")
            return

        # 比較跟上次是否一樣
        if (vx_raw, vy_raw, wz_raw) != self._last_tx_v:
            self.send_cmdvel(vx_raw, vy_raw, wz_raw)


    # # --- 統一的送出流程（帶一致性檢查） ---
    # def send_last_cmd(self):
    #     raw = self._last_cmd
    #     vx_raw, vy_raw, wz_raw = raw.linear.x, raw.linear.y, raw.angular.z

    #     # 限幅檢查
    #     if abs(vx_raw) > 50 or abs(vy_raw) > 50 or abs(wz_raw) > 50:
    #         self.get_logger().warn(f"[LIMIT] Reject cmd: vx={vx_raw:.3f}, vy={vy_raw:.3f}, wz={wz_raw:.3f}")
    #         return

    #     # 直接送出
    #     self.send_cmdvel(vx_raw, vy_raw, wz_raw)

    def send_cmdvel(self, vx: float, vy: float, wz: float):
        prefix = self.get_parameter('vel_prefix').value  # e.g. "V"
        line = f"{prefix} {vx:.4f} {vy:.4f} {wz:.4f}\n"
        try:
            self.ser.write(line.encode('ascii'))
        except Exception as e:
            self.get_logger().error(f"Serial write failed: {e}")
            return
        self.get_logger().info(f"[BASE] vx={vx:.3f}, vy={vy:.3f}, wz={wz:.3f}")
        self._last_tx_v = (vx, vy, wz)

    # ===================== B) PointStamped → 手臂（位置） =====================
    def cb_pointstamped(self, msg: PointStamped):
        # 只取座標，不用 header 的時間與 frame_id
        x = float(msg.point.x)
        y = float(msg.point.y)
        z = float(msg.point.z)
        self.send_arm_target(x, y, z)


    def cb_vision_result(self, msg: Int32):
        val = msg.data
        prefix = self.get_parameter('mode_prefix').value  # e.g. "MA"
        line = f"{prefix}{val}\n"
        try:
            self.ser.write(line.encode('ascii'))
            self.get_logger().info(f"[VISION->STM] {line.strip()}")
        except Exception as e:
            self.get_logger().error(f"Serial write failed (vision): {e}")

        
    def send_arm_target(self, x: float, y: float, z: float):
        prefix = self.get_parameter('arm_prefix').value  # e.g. "P"
        # 建議用不同前綴與底盤速度區分，避免 STM 解析混淆
        line = f"{prefix} {x:.4f} {y:.4f} {z:.4f}\n"
        try:
            self.ser.write(line.encode('ascii'))
        except Exception as e:
            self.get_logger().error(f"Serial write failed: {e}")
            return
        self.get_logger().info(f"[ARM] x={x:.3f}, y={y:.3f}, z={z:.3f}")

    # ===================== C) 背景讀取 STM 回傳 =====================
    def _read_loop(self):
        try:
            time.sleep(0.1)
            self.ser.reset_input_buffer()
        except Exception:
            pass

        while not self._stop:
            try:
                chunk = self.ser.read(64)
                if not chunk:
                    continue
                with self._rx_lock:
                    self._rx_buf.extend(chunk)
                    while True:
                        idx = self._rx_buf.find(b'\n')
                        if idx < 0:
                            break
                        line = self._rx_buf[:idx]     # 不含 '\n'
                        del self._rx_buf[:idx+1]
                        s = line.decode('ascii', errors='ignore').strip()
                        if s:
                            self._handle_line(s)
            except Exception:
                # 讀錯就略過
                pass

    def _handle_line(self, s: str):
        """
        解析 STM 回傳：
        1) "OK vx vy wz"  → /stm_vel
        2) "PA x y z"     → log
        3) "MODE N"       → /mode = N，再馬上 /mode = 0
        """
        s = s.strip()
        if not s:
            return
        
        # Debug print
        # self.get_logger().info(f"[RAW RX] {s}")

        s_norm = s.replace(',', ' ')

        # ---- 1) Strict velocity ACK: "OK vx vy wz" --------
        m = self._ok_line_re.match(s_norm)
        if m:
            try:
                vx = float(m.group(1))
                vy = float(m.group(2))
                wz = float(m.group(3))
            except ValueError:
                return

            # 限幅檢查
            if (abs(vx) > self._vx_abs_max) or (abs(vy) > self._vy_abs_max) or (abs(wz) > self._wz_abs_max):
                return

            # 與上次一致性檢查
            lvx, lvy, lwz = self._last_tx_v
            def near(a, b, eps): return abs(a - b) <= eps
            if not (near(vx, lvx, self._ack_eps) and near(vy, lvy, self._ack_eps) and near(wz, lwz, self._ack_eps)):
                return

            # 連續一致性檢查
            if self._ack_prev is not None:
                pvx, pvy, pwz = self._ack_prev
                if not (near(vx, pvx, self._ack_eps) and near(vy, pvy, self._ack_eps) and near(wz, pwz, self._ack_eps)):
                    self._ack_prev = (vx, vy, wz)
                    return
            self._ack_prev = (vx, vy, wz)

            # 發佈到 /stm_vel
            t = Twist(); t.linear.x = vx; t.linear.y = vy; t.angular.z = wz
            self.pub_vel.publish(t)
            self.get_logger().info_throttle(self.get_clock(), 0.5,
                f"STM ACK vel -> vx={vx:.3f}, vy={vy:.3f}, wz={wz:.3f}")
            return

        # ---- 2) Optional arm ACK: "PA x y z" --------
        m2 = self._pa_line_re.match(s_norm)
        if m2:
            ax = float(m2.group(1)); ay = float(m2.group(2)); az = float(m2.group(3))
            self.get_logger().info_throttle(self.get_clock(), 0.5,
                f"STM ACK arm -> x={ax:.3f}, y={ay:.3f}, z={az:.3f}")
            return

        # ---- 3a) "MODE N" --------
        m_mode = self._mode_keyval_re.match(s_norm)
        if m_mode:
            try:
                val = int(m_mode.group(1))
            except ValueError:
                return

            if val in (1, 2, 3):
                if self._current_mode != val:   # 只觸發一次
                    self.pub_mode.publish(Int32(data=val))
                    self.get_logger().info(f"[MODE] {val}")
                    self.pub_mode.publish(Int32(data=0))
                    self._current_mode = 0
                else:
                    pass
                return

        # ---- 3b) "N" (單純整數) --------
        m_mode_int = self._mode_only_int_re.match(s_norm)
        if m_mode_int:
            try:
                val = int(m_mode_int.group(1))
            except ValueError:
                return

            if val in (1, 2, 3):
                if self._current_mode != val:
                    self.pub_mode.publish(Int32(data=val))
                    self.get_logger().info(f"[MODE] {val}")
                    self.pub_mode.publish(Int32(data=0))
                self._current_mode = 0
            return


    # ===================== Shutdown =====================
    def destroy_node(self):
        self._stop = True
        try:
            self._reader.join(timeout=0.2)
        except Exception:
            pass
        return super().destroy_node()

def main():
    rclpy.init()
    node = Bridge()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
