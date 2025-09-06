#!/usr/bin/env python3
import rclpy, threading, re, time
from rclpy.node import Node
from geometry_msgs.msg import Twist, PointStamped
from std_msgs.msg import Int32, String
import serial

_FLOAT = r'[-+]?\d+(?:\.\d+)?'
_INT   = r'[-+]?\d+'

class Bridge(Node):
    def __init__(self):
        super().__init__('bridge')

        # ================= Ports / params =================
        self.declare_parameter('arm_port',  '/dev/ttyACM0')     # 手臂
        self.declare_parameter('arm_baud',  115200)
        self.declare_parameter('base_port', '/dev/ttyACM1')     # 底盤
        self.declare_parameter('base_baud', 115200)

        self.declare_parameter('mode_topic', 'mode')
        self.declare_parameter('ack_eps', 0.5)   # OK vx vy wz 與最近一次下發相比的容許誤差
        self.declare_parameter('vx_abs_max', 50.0)
        self.declare_parameter('vy_abs_max', 50.0)
        self.declare_parameter('wz_abs_max', 10.0)

        arm_port  = self.get_parameter('arm_port').value
        arm_baud  = int(self.get_parameter('arm_baud').value)
        base_port = self.get_parameter('base_port').value
        base_baud = int(self.get_parameter('base_baud').value)

        # ================= Open serials =================
        try:
            self.ser_arm = serial.Serial(
                arm_port, arm_baud, timeout=0.05, write_timeout=0.2,
                rtscts=False, dsrdtr=False
            )
            self.get_logger().info(f'Opened ARM  : {arm_port} @ {arm_baud}')
        except Exception as e:
            self.ser_arm = None
            self.get_logger().error(f'Open ARM serial failed: {e}')

        try:
            self.ser_base = serial.Serial(
                base_port, base_baud, timeout=0.05, write_timeout=0.2,
                rtscts=False, dsrdtr=False
            )
            self.get_logger().info(f'Opened BASE : {base_port} @ {base_baud}')
        except Exception as e:
            self.ser_base = None
            self.get_logger().error(f'Open BASE serial failed: {e}')
            raise SystemExit('Base serial is required for /cmd_vel, exiting.')

        # ================= ROS I/O =================
        self.create_subscription(Twist, '/cmd_vel', self.cb_cmd, 10)
        self.create_subscription(PointStamped, '/point_out', self.cb_point, 10)
        self.create_subscription(Int32, 'vision_result', self.cb_vision_result, 10)
        self.create_subscription(String, '/reset_cmd', self.cb_reset_cmd, 10)

        self.pub_vel = self.create_publisher(Twist, '/stm_vel', 10)  # 來自底盤的 OK vx vy wz
        self.pub_mode = self.create_publisher(Int32, self.get_parameter('mode_topic').value, 10)

        # ================= State =================
        self._last_tx_v = (0.0, 0.0, 0.0)   # 最近送到底盤的 (vx,vy,wz)
        self._ack_eps   = float(self.get_parameter('ack_eps').value)
        self._vx_abs_max = float(self.get_parameter('vx_abs_max').value)
        self._vy_abs_max = float(self.get_parameter('vy_abs_max').value)
        self._wz_abs_max = float(self.get_parameter('wz_abs_max').value)

        # 觸發型流程：MODE 1 -> 等待 vision_result 發 M*；MODE 2 -> 等待 point_out 發 S*
        self._mode_change = -1
        self._last_point = (0.0, 0.0, 0.0)

        # Regex：OK 與 MODE
        self._ok_line_re   = re.compile(rf'^\s*OK\s+({_FLOAT})\s+({_FLOAT})\s+({_FLOAT})\s*$')
        self._mode_line_re = re.compile(rf'^\s*mode[:\s]+({_INT})\s*$', re.IGNORECASE)

        # ================= Reader thread (BASE only) =================
        self._stop = False
        self._rx_lock = threading.Lock()
        self._rx_buf = bytearray()
        self._reader_base = threading.Thread(target=self._read_loop_base, daemon=True)
        self._reader_base.start()

        # 主動送 0 速度一次，避免殘值
        self.send_cmdvel(0.0, 0.0, 0.0)

        self.get_logger().info('Bridge ready: /cmd_vel/reset -> BASE (ACM1), M/S/A1 -> ARM (ACM0)')
        self.pub_mode.publish(Int32(data=0))
        self.get_logger().info("[MODE INIT] 0")

    # ================= Serial helpers =================
    def _send_to_arm(self, line: str):
        if not self.ser_arm or not self.ser_arm.is_open:
            self.get_logger().warn(f'[BRIDGE->ARM] drop (port not open): {line.strip()}')
            return
        try:
            self.ser_arm.write(line.encode('ascii'))
            self.get_logger().info(f'[BRIDGE->ARM] {line.strip()}')
        except Exception as e:
            self.get_logger().error(f'[BRIDGE->ARM] write failed: {e}')

    def _send_to_base(self, line: str):
        if not self.ser_base or not self.ser_base.is_open:
            self.get_logger().warn(f'[BRIDGE->BASE] drop (port not open): {line.strip()}')
            return
        try:
            self.ser_base.write(line.encode('ascii'))
            # self.get_logger().info(f'[BRIDGE->BASE] {line.strip()}')
        except Exception as e:
            self.get_logger().error(f'[BRIDGE->BASE] failed: {e}')

    # ================= ROS Callbacks =================
    def cb_cmd(self, msg: Twist):
        # 唯一會送到底盤（ACM1）的 ROS 指令
        vx, vy, wz = msg.linear.x, msg.linear.y, msg.angular.z
        # 你若需要單位轉換，請在這裡改，例如 m/s->cm/s；本版原樣轉發
        self.send_cmdvel(vx, vy, wz)

        if abs(vx) > 50 or abs(vy) > 50 or abs(wz) > 50:
            self.get_logger().warn(f"[LIMIT] Reject cmd: vx={vx:.3f}, vy={vy:.3f}, wz={wz:.3f}")
            return

    def send_cmdvel(self, vx: float, vy: float, wz: float):
        line = f'V {vx:.4f} {vy:.4f} {wz:.4f}\n'
        self._send_to_base(line)
        self._last_tx_v = (vx, vy, wz)
        self.get_logger().info(f"[BASE] vx={vx:.3f}, vy={vy:.3f}, wz={wz:.3f}")

    def cb_point(self, msg: PointStamped):
        # 只在 _mode_change==2（你原本流程）時，依點位送 S x y z 給手臂
        self._last_point = (msg.point.x, msg.point.y, msg.point.z)
        if self._mode_change == 2:
            x, y, z = self._last_point
            # 你原本 sample 有覆寫固定值，若要保留可改回下一行
            # x_i, y_i, z_i = int(x), int(y), int(z)
            x_i, y_i, z_i = int(x), int(y), int(z)
            self._send_to_arm(f'S{x_i} {y_i} {z_i}\n')
            self.get_logger().info(f"[S POINT] x={x_i}, y={y_i}, z={z_i}")
            self._mode_change = -1

    def cb_vision_result(self, msg: Int32):
        # 只在 _mode_change==1（你原本流程）時，把 vision 數字包成 M{n} 給手臂
        if self._mode_change == 1:
            val = int(msg.data)
            self._send_to_arm(f'M{val}\n')
            self.get_logger().info(f"[VISION->STM] M{val}")
            self._mode_change = -1

    def cb_reset_cmd(self, msg: String):
        # reset 類命令直接送手臂
        line = (msg.data.strip() + '\n')
        self._send_to_base(line)
        self.get_logger().info(f"[RESET CMD] {line.strip()}")

    # ================= BASE reader & parser =================
    def _read_loop_base(self):
        try:
            time.sleep(0.1)
            if self.ser_base:
                self.ser_base.reset_input_buffer()
        except Exception:
            pass

        while not self._stop:
            try:
                chunk = self.ser_base.read(64)
                if not chunk:
                    continue
                with self._rx_lock:
                    self._rx_buf.extend(chunk)
                    while True:
                        idx = self._rx_buf.find(b'\n')
                        if idx < 0:
                            break
                        raw = self._rx_buf[:idx]  # not include '\n'
                        del self._rx_buf[:idx+1]
                        s = raw.decode('ascii', errors='ignore').strip()
                        if s:
                            self._handle_line_from_base(s)
            except Exception as e:
                # 安全起見不中斷
                pass

    def _handle_line_from_base(self, s: str):
        # 1) 底盤回 A1 -> 立即轉發給手臂
        if s == 'A1':
            # self.get_logger().info('[BRIDGE RECEIVE] A1')
            self._send_to_arm('A1\n')
            return

        s_norm = s.replace(',', ' ')

        # 2) OK vx vy wz -> /stm_vel
        m_ok = self._ok_line_re.match(s_norm)
        if m_ok:
            try:
                vx = float(m_ok.group(1)); vy = float(m_ok.group(2)); wz = float(m_ok.group(3))
            except ValueError:
                return

            # 限幅與近似上次指令（可依需求關閉）
            if abs(vx) > self._vx_abs_max or abs(vy) > self._vy_abs_max or abs(wz) > self._wz_abs_max:
                return
            lvx, lvy, lwz = self._last_tx_v
            near = lambda a,b,eps: abs(a-b) <= eps
            if not (near(vx, lvx, self._ack_eps) and near(vy, lvy, self._ack_eps) and near(wz, lwz, self._ack_eps)):
                # 若不想驗證，直接註解掉這段 near 檢查
                pass

            t = Twist(); t.linear.x = vx; t.linear.y = vy; t.angular.z = wz
            self.pub_vel.publish(t)
            try:
                self.get_logger().info_throttle(self.get_clock(), 0.5,
                    f'STM ACK vel -> vx={vx:.3f}, vy={vy:.3f}, wz={wz:.3f}'
                )
            except Exception:
                self.get_logger().info(
                    f"STM ACK vel -> vx={vx:.3f}, vy={vy:.3f}, wz={wz:.3f}")
            return

        # 3) MODE n -> 公佈到 mode topic，並設定 _mode_change（1:等 vision 發 M*，2:等 point 發 S*）
        m_mode = self._mode_line_re.match(s_norm)
        if m_mode:
            try:
                val = int(m_mode.group(1))
            except ValueError:
                return
            self.pub_mode.publish(Int32(data=val))
            self.get_logger().info(f'[MODE] {val}')
            self.pub_mode.publish(Int32(data=0))
            if val in (1, 2, 3):
                self._mode_change = val
                self.get_logger().info(f"[MODE CHANGE] {val}")
            return

        # 其他訊息可視需要增加 parser
        # self.get_logger().info(f'[BASE RX] {s}')

    # ================= Shutdown =================
    def destroy_node(self):
        self._stop = True
        try:
            # 等一下 reader thread 收尾
            time.sleep(0.1)
        except Exception:
            pass
        try:
            if self.ser_arm:  self.ser_arm.close()
        except Exception:
            pass
        try:
            if self.ser_base: self.ser_base.close()
        except Exception:
            pass
        return super().destroy_node()

def main():
    rclpy.init()
    node = Bridge()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
