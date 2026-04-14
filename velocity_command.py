#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, TransformStamped
from nav_msgs.msg import Odometry
import serial, math, time
import tf2_ros

# ===== CONFIG =====
LEFT_IDS  = [2, 5]
RIGHT_IDS = [3, 7]

MOTOR_INVERT = {
    2: True,
    5: True,
    3: False,
    7: True
}

MOTOR_GAIN = {
    2: 1.00,
    5: 0.75,
    3: 0.90,
    7: 0.95
}

HEADING_KP = 2.5
CTE_KP     = 0.5
MAX_W      = 0.5

YAW_DRIFT_BIAS       = 0.012
CORRECTION_DIST      = 0.5
CORRECTION_THRESHOLD = math.radians(1.5)

WHEEL_BASE     = 0.35
WHEEL_RADIUS   = 0.0625
GEAR_RATIO     = 573.0
COUNTS_PER_REV = 334 * 4 * GEAR_RATIO

RAMP_STEP      = 300
MIN_DRIVE_RPM  = 700
SAFE_RPM_LIMIT = 8000

BAUD      = 9600
RPM_SCALE = 10000

CW_ENABLE  = 257
CCW_ENABLE = 265
CW_BRAKE   = 260
CCW_BRAKE  = 268
STOP_HOLD  = 1793


class RMCSBridge(Node):

    def __init__(self):
        super().__init__('rmcs_bridge_4enc')

        # ✅ FIX 1: Increased timeout
        self.right_ser = serial.Serial('/dev/ttyUSB0', BAUD, timeout=0.2)
        self.left_ser  = serial.Serial('/dev/ttyUSB1', BAUD, timeout=0.2)

        self.target_l  = 0.0
        self.target_r  = 0.0
        self.current_l = 0.0
        self.current_r = 0.0

        self.x = 0.0
        self.y = 0.0
        self.yaw = 0.0

        self.desired_yaw = 0.0
        self.start_y     = None
        self._moving     = False
        self._dist_since_check = 0.0

        self.last_dir = {2: 1, 5: 1, 3: 1, 7: 1}
        self.prev_pos = {2: None, 5: None, 3: None, 7: None}

        self.odom_pub = self.create_publisher(Odometry, '/odom', 10)
        self.tf_broadcaster = tf2_ros.TransformBroadcaster(self)
        self.create_subscription(Twist, '/cmd_vel', self.cmd_callback, 10)

        self.create_timer(0.05, self.ramp_loop)
        self.create_timer(0.15, self.odom_loop)  # ✅ slower odom loop

        self.get_logger().info(
            f'RMCS Bridge — 4 encoder motors ✓  '
            f'Wheel base: {WHEEL_BASE}m  Max: {SAFE_RPM_LIMIT} base RPM'
        )

    # ─── Modbus ─────────────────────────────────────────────

    def lrc(self, data):
        return ((-sum(data)) & 0xFF)

    def _frame(self, data):
        return (":" + "".join(f"{b:02X}" for b in data)
                + f"{self.lrc(data):02X}\r\n").encode()

    def write_register(self, ser, sid, addr, val):
        val = int(val)
        ser.write(self._frame([
            sid, 6,
            (addr >> 8) & 0xFF, addr & 0xFF,
            (val  >> 8) & 0xFF, val  & 0xFF
        ]))
        time.sleep(0.01)
        ser.read_all()

    def read_register(self, ser, sid, addr):
        ser.reset_input_buffer()
        ser.write(self._frame([sid, 3, (addr>>8)&0xFF, addr&0xFF, 0, 1]))

        raw = ser.read_until(b'\n').decode('ascii', errors='ignore').strip()

        # ✅ DEBUG LOG
        self.get_logger().info(f"RAW[{sid}] -> {raw}")

        if not raw.startswith(':') or len(raw) < 11:
            return None

        try:
            body = raw[1:]
            val = (int(body[6:8], 16) << 8) | int(body[8:10], 16)
            return val if val < 0x8000 else val - 0x10000
        except:
            return None

    def read_position(self, ser, sid):
        lsb = self.read_register(ser, sid, 20)
        if lsb is None:
            return None

        time.sleep(0.02)  # ✅ FIX delay

        msb = self.read_register(ser, sid, 22)
        if msb is None:
            return None

        raw = ((msb & 0xFFFF) << 16) | (lsb & 0xFFFF)

        if raw >= 0x80000000:
            raw -= 0x100000000

        return raw

    # ─── Motor Control ──────────────────────────────────────

    def _stop_motor(self, ser, sid):
        brake = CW_BRAKE if self.last_dir.get(sid, 1) >= 0 else CCW_BRAKE
        self.write_register(ser, sid, 2, brake)
        time.sleep(0.01)
        self.write_register(ser, sid, 2, STOP_HOLD)

    def _drive_motor(self, ser, sid, rpm):
        rpm = int(max(min(rpm, SAFE_RPM_LIMIT), -SAFE_RPM_LIMIT))
        rpm = int(rpm * MOTOR_GAIN[sid])

        if MOTOR_INVERT[sid]:
            rpm = -rpm

        if abs(rpm) < 50:
            self._stop_motor(ser, sid)
            return

        rpm = int(math.copysign(max(abs(rpm), MIN_DRIVE_RPM), rpm))

        self.last_dir[sid] = 1 if rpm >= 0 else -1

        self.write_register(ser, sid, 14, abs(rpm))
        time.sleep(0.01)
        self.write_register(ser, sid, 2,
                            CW_ENABLE if rpm >= 0 else CCW_ENABLE)

    def _stop_all(self):
        for mid in LEFT_IDS:
            self._stop_motor(self.left_ser, mid)
        for mid in RIGHT_IDS:
            self._stop_motor(self.right_ser, mid)

        self.current_l = self.current_r = 0.0
        self.target_l  = self.target_r  = 0.0

    # ─── CMD ───────────────────────────────────────────────

    def cmd_callback(self, msg):
        v = msg.linear.x
        w = msg.angular.z

        if abs(v) < 0.01:
            self._stop_all()
            return

        self.target_l = (v - w * WHEEL_BASE / 2.0) * RPM_SCALE
        self.target_r = (v + w * WHEEL_BASE / 2.0) * RPM_SCALE

    # ─── Ramp ──────────────────────────────────────────────

    def _ramp(self, cur, tgt):
        diff = tgt - cur
        if abs(diff) <= RAMP_STEP:
            return tgt
        return cur + math.copysign(RAMP_STEP, diff)

    def ramp_loop(self):
        self.current_l = self._ramp(self.current_l, self.target_l)
        self.current_r = self._ramp(self.current_r, self.target_r)

        for mid in LEFT_IDS:
            self._drive_motor(self.left_ser, mid, self.current_l)
        for mid in RIGHT_IDS:
            self._drive_motor(self.right_ser, mid, self.current_r)

    # ─── ODOM ──────────────────────────────────────────────

    def odom_loop(self):

        # ✅ FIX 2: Increased delays
        p_fl = self.read_position(self.left_ser, 2)
        time.sleep(0.03)
        p_rl = self.read_position(self.left_ser, 5)

        time.sleep(0.03)
        p_fr = self.read_position(self.right_ser, 3)
        time.sleep(0.03)
        p_rr = self.read_position(self.right_ser, 7)

        if None in (p_fl, p_rl, p_fr, p_rr):
            self.get_logger().warn('Encoder read failed — skipping odom')
            return

        if self.prev_pos[2] is None:
            self.prev_pos = {2: p_fl, 5: p_rl, 3: p_fr, 7: p_rr}
            self.get_logger().info('Encoders initialised ✓')
            return

        d_fl = p_fl - self.prev_pos[2]
        d_rl = p_rl - self.prev_pos[5]
        d_fr = p_fr - self.prev_pos[3]
        d_rr = p_rr - self.prev_pos[7]

        self.prev_pos = {2: p_fl, 5: p_rl, 3: p_fr, 7: p_rr}

        d_left  = -((d_fl + d_rl) / 2.0)
        d_right =   (d_fr + d_rr) / 2.0

        left_m  = (d_left  / COUNTS_PER_REV) * 2 * math.pi * WHEEL_RADIUS
        right_m = (d_right / COUNTS_PER_REV) * 2 * math.pi * WHEEL_RADIUS

        ds   = (left_m + right_m) / 2.0
        dyaw = (right_m - left_m) / WHEEL_BASE

        self.x += ds * math.cos(self.yaw + dyaw / 2.0)
        self.y += ds * math.sin(self.yaw + dyaw / 2.0)
        self.yaw += dyaw

        self.get_logger().info(
            f'ODOM x={self.x:.2f} y={self.y:.2f} yaw={math.degrees(self.yaw):.1f}'
        )


def main():
    rclpy.init()
    rclpy.spin(RMCSBridge())
    rclpy.shutdown()


if __name__ == '__main__':
    main()
