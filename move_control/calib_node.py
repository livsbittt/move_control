#!/usr/bin/env python3
"""Auto and manual calibration for wander safety.

Manual:  /calib/step  floor | cliff | compute
Auto:    /calib/step  auto
         wait stable floor IR (skip 4095)
         sample floor + IMU rest
         slow +x nudge → cmd_linear_sign + lidar_yaw_offset
         wait for a real IR cliff (nose over edge, not lift)
         save yaml and apply to safety_node
Lidar:   /calib/step  lidar
Abort:   /calib/step  abort
"""
import math
import os
import statistics

import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rcl_interfaces.msg import Parameter, ParameterType, ParameterValue
from rcl_interfaces.srv import SetParameters
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Imu, LaserScan, Range
from std_msgs.msg import String, UInt16MultiArray

from move_control.safety_node import parse_us_range, roll_pitch, sector_min


def yaw_from_quat(q) -> float:
    siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny_cosp, cosy_cosp)


def ir_valid(sample):
    return tuple(int(v) for v in sample if 50 < int(v) < 4000)


def looks_floor(sample) -> bool:
    v = ir_valid(sample)
    return bool(v) and len(v) >= 2 and min(v) >= 1500


def looks_cliff(sample, floor_m) -> bool:
    v = ir_valid(sample)
    if not v:
        return False
    return min(v) < min(1100.0, 0.40 * float(floor_m))


def _copy_scan(msg: LaserScan):
    return (
        float(msg.angle_min),
        float(msg.angle_increment),
        [float(r) for r in msg.ranges],
        float(msg.range_min),
        float(msg.range_max),
    )


def approach_heading(s0, s1):
    if s0 is None or s1 is None:
        return None
    amin, ainc, r0, rmin, rmax = s0
    _, _, r1, _, _ = s1
    n = min(len(r0), len(r1))
    sx = sy = wsum = 0.0
    hi = min(rmax, 6.0)
    for i in range(n):
        v0, v1 = r0[i], r1[i]
        if not (math.isfinite(v0) and math.isfinite(v1)):
            continue
        if not (rmin < v0 < hi and rmin < v1 < hi):
            continue
        dr = v1 - v0
        if dr >= -0.006:
            continue
        w = -dr
        ang = amin + i * ainc
        sx += w * math.cos(ang)
        sy += w * math.sin(ang)
        wsum += w
    if wsum < 0.012:
        return None
    return math.atan2(sy, sx)


def snap_lidar_yaw(ang: float) -> float:
    a = math.atan2(math.sin(ang), math.cos(ang))
    if abs(a) < math.radians(40.0):
        return 0.0
    if abs(abs(a) - math.pi) < math.radians(40.0):
        return math.pi
    return a


class CalibNode(Node):
    def __init__(self):
        super().__init__('calib_node')
        self.declare_parameter(
            'save_path',
            '/home/pinky/dev_ws/wj/src/move_control/config/cliff_calib.yaml',
        )
        self.declare_parameter(
            'sign_path',
            '/home/pinky/dev_ws/wj/src/move_control/config/auto_calib.yaml',
        )
        self.declare_parameter('ir_topic', '/ir_sensor/range')
        self.declare_parameter('samples', 30)
        self.declare_parameter('stable_hits', 8)
        self.declare_parameter('nudge_sec', 1.20)
        self.declare_parameter('nudge_v', 0.012)
        self.declare_parameter('settle_sec', 0.45)
        self.declare_parameter('cliff_wait_sec', 60.0)
        self.need = int(self.get_parameter('samples').value)

        self.pub_status = self.create_publisher(String, '/calib/status', 10)
        self.pub_phase = self.create_publisher(String, '/calib/phase', 10)
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.raw_pub = self.create_publisher(Twist, '/cmd_vel_raw', 10)
        self.wander_cmd = self.create_publisher(String, '/wander/cmd', 10)
        self.create_subscription(
            UInt16MultiArray, self.get_parameter('ir_topic').value, self.on_ir, 10
        )
        self.create_subscription(String, '/calib/step', self.on_step, 10)
        self.create_subscription(Odometry, '/odom', self.on_odom, 10)
        self.create_subscription(
            LaserScan, '/scan', self.on_scan, qos_profile_sensor_data
        )
        self.create_subscription(Range, '/us_sensor/range', self.on_us, 10)
        self.create_subscription(Imu, '/imu_raw', self.on_imu, qos_profile_sensor_data)
        self.create_timer(1.0, self._heartbeat)
        self.create_timer(0.05, self._tick)

        self.last_ir = None
        self.collecting = None
        self.buf = []
        self.sets = {'floor': None, 'line': None, 'cliff': None}
        self.phase = 'idle'
        self.phase_t0 = None
        self.linear_sign = None
        self.have_odom = False
        self.x = self.y = self.yaw = 0.0
        self.nudge_x = self.nudge_y = self.nudge_yaw = 0.0
        self.imu_buf = []
        self.scan_buf = []
        self.us_buf = []
        self.last_scan = None
        self.scan_before = None
        self.scan_after = None
        self.lidar_yaw = None
        self.stable_n = 0
        self.get_logger().info(
            'calib_node ready | auto (wait floor → nudge → cliff) | lidar | abort'
        )

    def now(self):
        return self.get_clock().now()

    def elapsed(self) -> float:
        if self.phase_t0 is None:
            return 0.0
        return (self.now() - self.phase_t0).nanoseconds * 1e-9

    def _set_phase(self, phase: str):
        self.phase = phase
        self.phase_t0 = self.now()
        self.pub_phase.publish(String(data=phase))

    def _heartbeat(self):
        if self.last_ir is None:
            self.get_logger().warn('IR 없음 — ros2 run pinky_sensor_adc main_node')
            return
        extra = f' phase={self.phase}' if self.phase != 'idle' else ''
        self.get_logger().info(f'IR now={self.last_ir}{extra}')

    def _status(self, text: str):
        self.get_logger().info(text)
        self.pub_status.publish(String(data=text))

    def _stop_motors(self):
        z = Twist()
        self.cmd_pub.publish(z)
        self.raw_pub.publish(z)

    def _drive(self, vx: float):
        cmd = Twist()
        cmd.linear.x = float(vx)
        self.cmd_pub.publish(cmd)
        self.raw_pub.publish(cmd)

    def on_odom(self, msg: Odometry):
        p = msg.pose.pose.position
        self.x, self.y = p.x, p.y
        self.yaw = yaw_from_quat(msg.pose.pose.orientation)
        self.have_odom = True

    def on_scan(self, msg: LaserScan):
        self.last_scan = _copy_scan(msg)
        if self.phase in ('auto_floor', 'auto_wait_floor') or self.collecting == 'floor':
            d = sector_min(msg, 0.0, math.radians(22.0))
            if math.isfinite(d) and d > 0.0:
                self.scan_buf.append(d)

    def on_us(self, msg: Range):
        if self.phase not in ('auto_floor', 'auto_wait_floor') and self.collecting != 'floor':
            return
        parsed = parse_us_range(msg)
        if parsed is not None:
            self.us_buf.append(parsed)

    def on_imu(self, msg: Imu):
        if self.phase not in ('auto_floor', 'auto_wait_floor') and self.collecting != 'floor':
            return
        roll, pitch = roll_pitch(msg.orientation)
        self.imu_buf.append((roll, pitch))

    def on_ir(self, msg: UInt16MultiArray):
        if len(msg.data) < 3:
            return
        sample = tuple(int(v) for v in msg.data[:3])
        self.last_ir = sample
        if not self.collecting:
            return
        if self.collecting == 'floor' and not looks_floor(sample):
            return
        if self.collecting == 'cliff':
            floor_m = 2500.0
            if self.sets['floor']:
                floor_m = statistics.median(
                    [min(ir_valid(s) or (4095,)) for s in self.sets['floor']]
                )
            if not looks_cliff(sample, floor_m):
                return
        self.buf.append(sample)
        if len(self.buf) < self.need:
            return
        pose = self.collecting
        self.collecting = None
        samples = list(self.buf)
        self.buf = []
        self._accept_samples(pose, samples)

    def _accept_samples(self, pose, samples):
        mins = [min(ir_valid(s) or (4095,)) for s in samples]
        med = statistics.median(mins)
        if pose == 'cliff' and med > 1500:
            self.sets['cliff'] = None
            self._status(
                f'cliff 거부 min~{med:.0f} now={self.last_ir} '
                f'들지 말고 코만 책상 밖. 200~800'
            )
            if self.phase == 'auto_cliff':
                self._set_phase('auto_wait_cliff')
            return
        if pose == 'floor' and med < 1200:
            self.sets['floor'] = None
            self._status(f'floor 거부 min~{med:.0f} — 책상 가운데, 4095 아닌 바닥')
            if self.phase.startswith('auto'):
                self.collecting = None
                self.buf = []
                self.stable_n = 0
                self._set_phase('auto_wait_floor')
                self._status('AUTO 바닥 IR이 안정될 때까지 대기')
            return
        self.sets[pose] = samples
        self._status(f'{pose} ok n={len(mins)} min~{med:.0f} now={samples[-1]}')
        if self.phase == 'auto_floor' and pose == 'floor':
            self._begin_nudge()
        elif self.phase == 'auto_cliff' and pose == 'cliff':
            self.compute()
            self._set_phase('idle')
            self._stop_motors()

    def on_step(self, msg: String):
        step = msg.data.strip().lower()
        aliases = {
            'ground': 'floor', 'white': 'line', 'tape': 'line',
            'void': 'cliff', 'edge': 'cliff',
            'start': 'auto', 'autocal': 'auto', 'auto_cal': 'auto',
            'cancel': 'abort', 'stop': 'abort',
        }
        step = aliases.get(step, step)
        if step in ('abort',):
            self._abort('취소')
            return
        if step == 'auto':
            self._start_auto()
            return
        if step in ('lidar', 'lidar_yaw', 'scan'):
            self._begin_nudge()
            return
        if step in self.sets:
            if self.last_ir is None:
                self._status('IR 없음 — pinky_sensor_adc 를 먼저')
                return
            self.collecting = step
            self.buf = []
            self._status(f'{step} 샘플링 중 {self.last_ir}')
            return
        if step in ('compute', 'save', 'apply'):
            self.compute()
            return
        self._status('use auto | lidar | floor | cliff | compute | abort')

    def _start_auto(self):
        self.wander_cmd.publish(String(data='stop'))
        self._stop_motors()
        self.sets = {'floor': None, 'line': None, 'cliff': None}
        self.linear_sign = None
        self.lidar_yaw = None
        self.scan_before = None
        self.scan_after = None
        self.imu_buf = []
        self.scan_buf = []
        self.us_buf = []
        self.collecting = None
        self.buf = []
        self.stable_n = 0
        self._set_phase('auto_wait_floor')
        self._status('AUTO 시작 — 책상 가운데, 바닥 IR 안정 대기 (4095 무시)')

    def _begin_nudge(self):
        self._stop_motors()
        if not self.have_odom:
            self._status('odom 없음 — linear_sign 건너뜀, 라이다 요만 시도')
        self.nudge_x, self.nudge_y, self.nudge_yaw = self.x, self.y, self.yaw
        self.scan_before = self.last_scan
        self.scan_after = None
        v = float(self.get_parameter('nudge_v').value)
        self._set_phase('auto_nudge')
        self._drive(v)
        self._status(f'AUTO 느린 +x {v:.3f} m/s × {float(self.get_parameter("nudge_sec").value):.1f}s (방향+라이다)')

    def _begin_wait_cliff(self):
        self._stop_motors()
        self.stable_n = 0
        self._set_phase('auto_wait_cliff')
        lim = float(self.get_parameter('cliff_wait_sec').value)
        self._status(f'AUTO 코만 책상 밖으로 — 절벽 IR이 안정되면 샘플 ({lim:.0f}s)')

    def _abort(self, why: str):
        self.collecting = None
        self.buf = []
        self.stable_n = 0
        self._stop_motors()
        self._set_phase('idle')
        self._status(f'calib abort: {why}')

    def _tick(self):
        if self.phase == 'auto_wait_floor':
            self._tick_wait_floor()
        elif self.phase == 'auto_nudge':
            self._tick_nudge()
        elif self.phase == 'auto_settle':
            self._tick_settle()
        elif self.phase == 'auto_wait_cliff':
            self._tick_wait_cliff()
        elif self.phase == 'auto_floor' and self.elapsed() > 15.0 and not self.collecting:
            self._status('floor 샘플 느림 — 계속 대기 (4095는 건너뜀)')
            self._set_phase('auto_wait_floor')
            self.stable_n = 0

    def _tick_wait_floor(self):
        if self.elapsed() > 25.0:
            self._abort('바닥 IR 대기 시간초과 — 책상 가운데, 들어올리지 말 것')
            return
        need = int(self.get_parameter('stable_hits').value)
        if self.last_ir is not None and looks_floor(self.last_ir):
            self.stable_n += 1
        else:
            self.stable_n = 0
        if self.stable_n < need:
            return
        self.collecting = 'floor'
        self.buf = []
        self._set_phase('auto_floor')
        self._status(f'AUTO floor 샘플 {self.need}개 IR={self.last_ir}')

    def _tick_nudge(self):
        v = float(self.get_parameter('nudge_v').value)
        dur = float(self.get_parameter('nudge_sec').value)
        if self.elapsed() < dur:
            self._drive(v)
            return
        self._stop_motors()
        self._set_phase('auto_settle')

    def _tick_settle(self):
        self._stop_motors()
        if self.last_scan is not None:
            self.scan_after = self.last_scan
        if self.elapsed() < float(self.get_parameter('settle_sec').value):
            return
        dx = self.x - self.nudge_x
        dy = self.y - self.nudge_y
        ahead = dx * math.cos(self.nudge_yaw) + dy * math.sin(self.nudge_yaw)
        if not self.have_odom:
            ahead = 0.0
        if abs(ahead) < 0.003:
            self._status(f'AUTO 방향 불명 Δ={ahead:.3f}m — linear_sign 유지')
        elif ahead > 0.0:
            self.linear_sign = 1.0
            self._status(f'AUTO +x 가 전진 Δ={ahead:.3f}m → cmd_linear_sign=+1')
        else:
            self.linear_sign = -1.0
            self._status(f'AUTO +x 가 후진 Δ={ahead:.3f}m → cmd_linear_sign=-1')
        ap = approach_heading(self.scan_before, self.scan_after or self.last_scan)
        if ap is None:
            self._status('AUTO 라이다 요 불명 — 전방에 벽이 있으면 더 잘 잡힘')
        else:
            if ahead < -0.003:
                ap = math.atan2(math.sin(ap + math.pi), math.cos(ap + math.pi))
            self.lidar_yaw = snap_lidar_yaw(ap)
            self._status(
                f'AUTO lidar_yaw_offset={math.degrees(self.lidar_yaw):.0f}deg '
                f'(approach {math.degrees(ap):.0f}deg)'
            )
        if self.sets['floor']:
            self._begin_wait_cliff()
        else:
            self._set_phase('idle')
            self._status('lidar/방향만 측정됨 — cliff 는 auto 또는 cliff')

    def _tick_wait_cliff(self):
        lim = float(self.get_parameter('cliff_wait_sec').value)
        if self.elapsed() > lim:
            self._abort('cliff 대기 시간초과 — 코만 책상 밖 (들면 4095)')
            return
        if self.last_ir is None or not self.sets['floor']:
            return
        floor_m = statistics.median(
            [min(ir_valid(s) or (4095,)) for s in self.sets['floor']]
        )
        need = int(self.get_parameter('stable_hits').value)
        if looks_cliff(self.last_ir, floor_m):
            self.stable_n += 1
        else:
            self.stable_n = 0
        if self.stable_n < need:
            return
        self.collecting = 'cliff'
        self.buf = []
        self._set_phase('auto_cliff')
        self._status(f'AUTO cliff 안정 IR={self.last_ir} 샘플링')

    def compute(self):
        if not self.sets['floor'] or not self.sets['cliff']:
            miss = [p for p in ('floor', 'cliff') if not self.sets[p]]
            self._status(f'필수 없음: {miss}')
            return
        cliff_vals = [min(ir_valid(s) or (4095,)) for s in self.sets['cliff']]
        floor_vals = [min(ir_valid(s) or (4095,)) for s in self.sets['floor']]
        floor_m = statistics.median(floor_vals)
        cliff_m = statistics.median(cliff_vals)
        cliff_hi = sorted(cliff_vals)[int(0.9 * (len(cliff_vals) - 1))]
        floor_lo = sorted(floor_vals)[int(0.1 * (len(floor_vals) - 1))]
        if cliff_m > 1500:
            self._status(f'cliff={cliff_m:.0f} 잘못됨(들어올림). 코만 허공으로 다시')
            return
        if cliff_m >= floor_m:
            self._status(
                f'절벽이 바닥보다 밝음 cliff={cliff_m:.0f} floor={floor_m:.0f}. 다시'
            )
            return
        gap = floor_lo - cliff_hi
        if gap < 80:
            self._status(f'간격 부족 gap={gap:.0f} floor={floor_m:.0f} cliff={cliff_m:.0f}')
            return
        th = int(cliff_hi + 0.4 * gap)
        clear = int(th + 0.4 * (floor_m - th))
        self._status(
            f'OK mode=low cliff_raw_max={th} clear={clear} '
            f'floor={floor_m:.0f} cliff={cliff_m:.0f} gap={gap:.0f}'
        )
        yaml_text = (
            'safety_node:\n  ros__parameters:\n'
            '    cliff_enable: true\n'
            f'    cliff_raw_max: {th}\n    cliff_clear_raw: {clear}\n'
            '    cliff_mode: low\n    cliff_hits: 2\n'
        )
        self._write(self.get_parameter('save_path').value, yaml_text)
        apply_params = [
            ('cliff_enable', True),
            ('cliff_raw_max', th),
            ('cliff_clear_raw', clear),
            ('cliff_mode', 'low'),
            ('cliff_hits', 2),
        ]
        extra = {}
        if self.linear_sign is not None:
            extra['cmd_linear_sign'] = float(self.linear_sign)
            apply_params.append(('cmd_linear_sign', float(self.linear_sign)))
        if self.imu_buf:
            rolls = [r for r, _ in self.imu_buf]
            pitches = [p for _, p in self.imu_buf]
            extra['imu_roll0'] = math.degrees(statistics.median(rolls))
            extra['imu_pitch0'] = math.degrees(statistics.median(pitches))
            apply_params.append(('imu_roll0', extra['imu_roll0']))
            apply_params.append(('imu_pitch0', extra['imu_pitch0']))
            self._status(
                f'IMU rest roll={extra["imu_roll0"]:.1f} pitch={extra["imu_pitch0"]:.1f} deg'
            )
        extra['camera_as_wall'] = False
        extra['stop_distance'] = 0.03
        extra['clear_distance'] = 0.05
        extra['us_stop_distance'] = 0.025
        extra['us_clear_distance'] = 0.04
        extra['front_half_width_deg'] = 22.0
        apply_params.extend([
            ('camera_as_wall', False),
            ('stop_distance', 0.03),
            ('clear_distance', 0.05),
            ('us_stop_distance', 0.025),
            ('us_clear_distance', 0.04),
            ('front_half_width_deg', 22.0),
        ])
        if self.lidar_yaw is not None:
            extra['lidar_yaw_offset'] = float(self.lidar_yaw)
            apply_params.append(('lidar_yaw_offset', float(self.lidar_yaw)))
            self._status(f'lidar_yaw_offset={math.degrees(self.lidar_yaw):.0f}deg 저장')
        lines = ['safety_node:\n  ros__parameters:\n']
        for k, v in extra.items():
            if isinstance(v, bool):
                lines.append(f'    {k}: {str(v).lower()}\n')
            elif isinstance(v, float):
                lines.append(f'    {k}: {v:.4f}\n')
            else:
                lines.append(f'    {k}: {v}\n')
        self._write(self.get_parameter('sign_path').value, ''.join(lines))
        self._apply_safety(apply_params)
        self._status('AUTO 완료')

    def _write(self, path, text):
        try:
            os.makedirs(os.path.dirname(path) or '.', exist_ok=True)
            with open(path, 'w', encoding='utf-8') as f:
                f.write(text)
            self._status(f'saved {path}')
        except OSError as exc:
            self._status(f'save fail {exc}')

    def _apply_safety(self, pairs):
        client = self.create_client(SetParameters, '/safety_node/set_parameters')
        if not client.wait_for_service(timeout_sec=0.5):
            self._status('safety_node 없음 — yaml만 저장됨')
            return
        params = []
        for name, val in pairs:
            pv = ParameterValue()
            if isinstance(val, bool):
                pv.type = ParameterType.PARAMETER_BOOL
                pv.bool_value = val
            elif isinstance(val, int) and not isinstance(val, bool):
                pv.type = ParameterType.PARAMETER_INTEGER
                pv.integer_value = int(val)
            elif isinstance(val, float):
                pv.type = ParameterType.PARAMETER_DOUBLE
                pv.double_value = float(val)
            else:
                pv.type = ParameterType.PARAMETER_STRING
                pv.string_value = str(val)
            params.append(Parameter(name=name, value=pv))
        req = SetParameters.Request()
        req.parameters = params
        client.call_async(req)
        self._status('safety_node 파라미터 적용')


def main():
    rclpy.init()
    node = CalibNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node._stop_motors()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
