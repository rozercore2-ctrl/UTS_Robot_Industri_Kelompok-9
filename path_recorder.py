#!/usr/bin/env python3
import math
import os

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray


def merge(segs):
    out = []
    for k, v in segs:
        if out and out[-1][0] == k:
            out[-1] = (k, out[-1][1] + v)
        else:
            out.append((k, v))
    return out


class PathRecorder(Node):
    def __init__(self):
        super().__init__('path_recorder')
        self.declare_parameter('wheel_radius', 0.1)
        self.declare_parameter('wheel_sep', 0.45)
        self.declare_parameter('out_file', '~/segments.txt')
        self.r = self.get_parameter('wheel_radius').value
        self.L = self.get_parameter('wheel_sep').value
        self.out_file = os.path.expanduser(
            self.get_parameter('out_file').value)

        self.create_subscription(Float64MultiArray, '/wheel/speed',
                                 self.cb, 10)
        self.create_timer(1.0, self.status)
        self.clear()
        self.get_logger().info(
            'Perekam siap'
            'i=maju, j=putar kiri, l=putar kanan, k=berhenti. '
            'Di finish tekan k, lalu Ctrl+C di sini.')

    def clear(self):
        self.last_t = None
        self.cur_kind = None
        self.cur_val = 0.0
        self.raw = []
        self.dist = 0.0
        self.heading = 0.0
        self.warned_arc = False

    def now(self):
        return self.get_clock().now().nanoseconds / 1e9

    def flush(self):
        if self.cur_kind is not None:
            self.raw.append((self.cur_kind, self.cur_val))
        self.cur_kind = None
        self.cur_val = 0.0

    def cb(self, msg):
        t = self.now()
        if self.last_t is not None and t < self.last_t - 0.5:
            self.get_logger().info('Reset terdeteksi: rekaman dikosongkan.')
            self.clear()
        if self.last_t is None:
            self.last_t = t
            return
        dt = t - self.last_t
        self.last_t = t
        if dt <= 0.0 or dt > 0.2:
            return

        wl, wr = msg.data[0], msg.data[1]
        v = self.r * (wl + wr) / 2.0
        w = self.r * (wr - wl) / self.L

        moving = abs(v) > 0.05
        turning = abs(w) > 0.15
        if not moving and not turning:
            return

        if moving and turning:
            if not self.warned_arc:
                self.get_logger().warn(
                    'Gerak melengkung terdeteksi. Pakai i/j/l/k saja.')
                self.warned_arc = True
            kind = 'F' if abs(v) / 0.3 >= abs(w) / 0.6 else 'T'
        else:
            kind = 'F' if moving else 'T'

        if kind != self.cur_kind:
            self.flush()
            self.cur_kind = kind
        if kind == 'F':
            self.cur_val += v * dt
            self.dist += v * dt
        else:
            self.cur_val += math.degrees(w * dt)
            self.heading += math.degrees(w * dt)

    def status(self):
        if self.last_t is None:
            return
        self.get_logger().info(
            f'jarak={self.dist:.2f} m  arah={self.heading:.0f} deg  '
            f'segmen={len(self.raw)}')

    def report(self):
        self.flush()
        segs = merge(self.raw)
        segs = [(k, v) for k, v in segs
                if not (k == 'F' and abs(v) < 0.15)]
        segs = merge(segs)
        segs = [(k, v) for k, v in segs
                if not (k == 'T' and abs(v) < 8.0)]
        segs = merge(segs)

        final = []
        for k, v in segs:
            if k == 'T':
                near = round(v / 90.0) * 90.0
                if near != 0 and abs(v - near) <= 25.0:
                    v = near
                else:
                    v = round(v / 5.0) * 5.0
                if v == 0:
                    continue
            else:
                v = round(v / 0.05) * 0.05
            final.append((k, v))

        lines = ['SEGMENTS = [']
        for k, v in final:
            lines.append(f"    ('F', {v:.2f})," if k == 'F'
                         else f"    ('T', {v:.0f}),")
        lines.append(']')
        text = '\n'.join(lines)

        total_f = sum(abs(v) for k, v in final if k == 'F')
        total_t = sum(abs(v) for k, v in final if k == 'T')
        print('\n' + '=' * 50)
        print('Salin ke track_runner.py:\n')
        print(text)
        print(f'\n{len(final)} segmen, jarak {total_f:.1f} m, '
              f'putar total {total_t:.0f} deg')
        if len(final) > 30:
            print('PERINGATAN: segmen terlalu banyak. Kemungkinan banyak '
                  'koreksi kecil. Pertimbangkan rekam ulang.')
        print('=' * 50)
        try:
            with open(self.out_file, 'w') as f:
                f.write(text + '\n')
            print(f'Juga tersimpan di {self.out_file}')
        except OSError as e:
            print(f'Gagal menyimpan file: {e}')


def main():
    rclpy.init()
    node = PathRecorder()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.report()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
