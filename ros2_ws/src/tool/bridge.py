# wcz_0819

#!/usr/bin/env python3
import rclpy, threading, re
import time
from rclpy.node import Node
from geometry_msgs.msg import Twist, PointStamped
from std_msgs.msg import Int32, Float64, String
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy
import serial

# type input int / float
_FLOAT = r'[-+]?\d+(?:\.\d+)?'
_INT   = r'[-+]?\d+'

class Bridge(Node):
    def __init__(self):
        super().__init__('Bridge')

        self.declare_parameter('port', '/dev/ttyACM0')
        self.declare_parameter('baud', 115200)

        self.declare_parameter('mode_topic', 'mode')

        port = self.get_parameter('port').value
        baud = int(self.get_parameter('baud').value)

        # ---------- Serial ----------
        try:
            self.ser = serial.Serial(
                port, baud, timeout=0.05, write_timeout=0.2,
                rtscts=False, dsrdtr=False
            )
        except Exception as e:
            raise SystemExit(f"Open serial failed: {e}")
        self.get_logger().info(f'Opened {port} @ {baud}')


        self.create_subscription(Twist, '/cmd_vel', self.cb_cmd, 10)
        self.create_subscription(PointStamped, '/point_out', self.cb_point, 10)

        self.create_subscription(Int32, "vision_result", self.cb_vision_result, 10)
        self.create_subscription(String, "/reset_cmd", self.cb_reset_cmd, 10)

        # 從 STM 回讀的速度 (若 STM 有回三個浮點數，仍轉成 Twist 發到 /stm_vel)
        self.pub_vel = self.create_publisher(Twist, '/stm_vel', 10)

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

        # ---------- Reader thread ----------
        self._rx_buf = bytearray()
        self._rx_lock = threading.Lock()
        self._stop = False
        self._reader = threading.Thread(target=self._read_loop, daemon=True)
        self._reader.start()

        self._last_cmd = Twist()       # 最近收到的原始指令（未轉換）

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

        # "mode 1" / "MODE:2"
        self._mode_keyval_re = re.compile(rf'^\s*mode[:\s]+({_INT})\s*$', re.IGNORECASE)

        # 連續一致性：連兩筆 ACK 近似才發佈（進一步抗雜訊）
        self._ack_prev = None

        # initial the situation of mode
        self._current_mode = 0
        self.pub_mode.publish(Int32(data=0))   # send 0 at the beginning
        self.get_logger().info("[MODE INIT] 0")

        self._mode_change = -1

        self._last_point = (0.0, 0.0, 0.0)  # last PointStamped

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

    # /cmd_vel
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

    def cb_point(self, msg: PointStamped):
        self._last_point = (msg.point.x, msg.point.y, msg.point.z)

        # ★ Mode 2：改成由 point 觸發一次性送 S
        if self._mode_change == 2:
            x, y, z = self._last_point
            x_i, y_i, z_i = int(x), int(y), int(z)
            x_i = 4
            y_i = 3
            z_i = 3
            line = f"S{x_i} {y_i} {z_i}\n"
            try:
                self.ser.write(line.encode('ascii'))
                self.get_logger().info(f"[S POINT] x={x_i}, y={y_i}, z={z_i}")
            except Exception as e:
                self.get_logger().error(f"Serial write failed (S via point): {e}")
            finally:
                self._mode_change = -1  # 只送一次

    def send_cmdvel(self, vx: float, vy: float, wz: float):
        line = f"V {vx:.4f} {vy:.4f} {wz:.4f}\n"
        try:
            self.ser.write(line.encode('ascii'))
        except Exception as e:
            self.get_logger().error(f"Serial write failed: {e}")
            return
        self.get_logger().info(f"[BASE] vx={vx:.3f}, vy={vy:.3f}, wz={wz:.3f}")
        self._last_tx_v = (vx, vy, wz)

    def cb_vision_result(self, msg: Int32):
        val = msg.data

        if self._mode_change == 1:
            # mode = 1, M1 ~ M8
            line = f"M{val}\n"
            try:
                self.ser.write(line.encode('ascii'))
                self.get_logger().info(f"[VISION->STM] {line.strip()}")
            except Exception as e:
                self.get_logger().error(f"Serial write failed (vision): {e}")
            finally:
                self._mode_change = -1
        elif self._mode_change == 2:
            # 交給 cb_point() 觸發送 S，這裡不動
            return
        else:
            return

    def cb_reset_cmd(self, msg: String):
        line = msg.data.strip() + "\n"
        try:
            self.ser.write(line.encode("ascii"))
            self.get_logger().info(f"[RESET->STM] {line.strip()}")
        except Exception as e:
            self.get_logger().error(f"Serial write failed (reset): {e}")

    # 背景讀取 STM 回傳
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

        s = s.strip()
        if not s:
            return

        s_norm = s.replace(',', ' ')

        # ---- Strict velocity ACK: "OK vx vy wz" --------
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

            # send to /stm_vel
            t = Twist(); t.linear.x = vx; t.linear.y = vy; t.angular.z = wz
            self.pub_vel.publish(t)
            self.get_logger().info_throttle(self.get_clock(), 0.5,
                f"STM ACK vel -> vx={vx:.3f}, vy={vy:.3f}, wz={wz:.3f}")
            return

        # ---- "MODE N" --------
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
                    self._mode_change = val
                    self.get_logger().info(f"[MODE CHANGE] {val}")
                else:
                    pass
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
