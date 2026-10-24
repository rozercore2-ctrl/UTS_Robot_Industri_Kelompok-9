#!/usr/bin/env python3
import math

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Float64MultiArray

# ===== RUTE ROBOT (, track kiri) =====
# ('F', meter)   = maju
# ('T', derajat) = putar di tempat, positif = kiri, negatif = kanan
SEGMENTS = [
    ('F', 2.50),
    ('T', 116),
    ('F', 7.85),
    ('T', -115),
    ('F', 5.60),
    ('T', 115),
    ('F', 2.50),
    ('T', -115),
    ('F', 4.50),
    ('T', -113),
    ('F', 7.80),
    ('T', 113),
    ('F', 14.10),
    ('T', 114),
    ('F', 18.50),
    ('T', 45.00),
    ('F', 3.00),
    ('T', 45),
    ('F', 2.70),
    ('T', 35),
    ('F', 2.70),
    ('T', 25),
    ('F', 2.60),
    ('T', 45),
    ('F', 3.00),
    ('T', 30),
    ('F', 5.80),
    ('T', -114),
    ('F', 2.60),
    ('T', -114),
    ('F', 8.00),
    ('T', 115),
    ('F', 8.00),
    ('T', -114),
    ('F', 2.50),
    ('T', 115),
    ('F', 4.80),
    ('T', 115),
    ('F', 9.50),

]
def clamp(x, lo, hi):
    return max(lo, min(hi, x))


class TrackRunner(Node):
    def __init__(self):
        super().__init__('track_runner')
        self.declare_parameter('wheel_radius', 0.1)    # hasil kalibrasi
        self.declare_parameter('wheel_sep', 0.45)      # hasil kalibrasi
        self.declare_parameter('v_max', 0.4)           # m/s
        self.declare_parameter('v_min', 0.05)
        self.declare_parameter('w_max', 1.5)           # rad/s
        self.declare_parameter('w_min', 0.15)
        self.declare_parameter('kp_v', 1.5)
        self.declare_parameter('kp_w', 2.0)
        self.declare_parameter('k_heading', 2.0)
        self.declare_parameter('pos_tol', 0.02)        # m
        self.declare_parameter('ang_tol', 0.02)        # rad (~1 derajat)

        p = self.get_parameter
        self.r = p('wheel_radius').value
        self.L = p('wheel_sep').value
        self.v_max = p('v_max').value
        self.v_min = p('v_min').value
        self.w_max = p('w_max').value
        self.w_min = p('w_min').value
        self.kp_v = p('kp_v').value
        self.kp_w = p('kp_w').value
        self.k_h = p('k_heading').value
        self.pos_tol = p('pos_tol').value
        self.ang_tol = p('ang_tol').value
        self.v_meas = 0.0       # kecepatan maju terukur (dari roda)
        self.w_meas = 0.0       # kecepatan putar terukur (dari roda)
        self.get_logger().info(
            f'PARAMETER: v_max={self.v_max} m/s  kp_v={self.kp_v}  '
            f'w_max={self.w_max} rad/s  kp_w={self.kp_w}')

        self.pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.create_subscription(Float64MultiArray, '/wheel/speed',
                                 self.wheel_cb, 10)

        # odometri dari kecepatan roda
        self.s = 0.0            # jarak tempuh (m)
        self.theta = 0.0        # arah hadap (rad)
        self.last_t = None
        self.have_wheel = False

        # state rute
        self.idx = -1
        self.target_theta = 0.0
        self.seg_s0 = 0.0
        self.start_time = None
        self.finished = False

        self.create_timer(0.02, self.loop)
        self.get_logger().info(
            f'Track runner siap ({len(SEGMENTS)} segmen), '
            'menunggu /wheel/speed...')

    def now(self):
        return self.get_clock().now().nanoseconds / 1e9

    def wheel_cb(self, msg):
        t = self.now()
        wl, wr = msg.data[0], msg.data[1]
        if self.last_t is not None:
            dt = t - self.last_t
            v = self.r * (wl + wr) / 2.0
            w = self.r * (wr - wl) / self.L
            self.v_meas = v
            self.w_meas = w
            self.s += v * dt
            self.theta += w * dt
        self.last_t = t
        self.have_wheel = True

    def next_segment(self):
        self.idx += 1
        if self.idx >= len(SEGMENTS):
            return False
        kind, val = SEGMENTS[self.idx]
        self.seg_s0 = self.s
        if kind == 'T':
            self.target_theta += math.radians(val)
        if kind == 'F':
            ket = f'MAJU {val} m'
        else:
            ket = f'BELOK {"KIRI" if val > 0 else "KANAN"} {abs(val)} deg'
        self.get_logger().info(f'Segmen {self.idx + 1}/{len(SEGMENTS)}: {ket}')
        return True

    def loop(self):
        t = self.now()
        if t == 0.0 or not self.have_wheel:
            return
        if self.finished:
            return

        if self.start_time is None:
            self.start_time = t
            self.get_logger().info('Start! Timer berjalan...')
            self.next_segment()

        kind, val = SEGMENTS[self.idx]
        cmd = Twist()

        if kind == 'F':
            direction = 1.0 if val >= 0 else -1.0
            remaining = abs(val) - direction * (self.s - self.seg_s0)
            if remaining <= self.pos_tol:
                if not self.next_segment():
                    return self.finish(t)
                return
            v = clamp(self.kp_v * remaining, self.v_min, self.v_max)
            cmd.linear.x = direction * v
            herr = self.target_theta - self.theta
            cmd.angular.z = clamp(self.k_h * herr, -1.0, 1.0)
        else:  # 'T'
            err = self.target_theta - self.theta
            if abs(err) <= self.ang_tol:
                if not self.next_segment():
                    return self.finish(t)
                return
            sign = 1.0 if err > 0 else -1.0
            w = clamp(self.kp_w * abs(err), self.w_min, self.w_max)
            cmd.angular.z = sign * w

        self.pub.publish(cmd)
        self.get_logger().info(
            f'[{self.idx + 1}/{len(SEGMENTS)} {kind} {val}] '
            f'perintah: v={cmd.linear.x:.2f} m/s w={math.degrees(cmd.angular.z):.0f} deg/s | '
            f'terukur: v={self.v_meas:.2f} m/s w={math.degrees(self.w_meas):.0f} deg/s | '
            f'jarak={self.s:.2f} m arah={math.degrees(self.theta):.1f} deg',
            throttle_duration_sec=1.0)

    def finish(self, t):
        self.finished = True
        self.pub.publish(Twist())
        elapsed = t - self.start_time
        self.get_logger().info('=' * 40)
        self.get_logger().info(f'FINISH! Waktu tempuh: {elapsed:.2f} detik')
        self.get_logger().info('=' * 40)


def main():
    rclpy.init()
    node = TrackRunner()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.pub.publish(Twist())
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
