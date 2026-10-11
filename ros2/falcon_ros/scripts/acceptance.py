#!/usr/bin/env python3
import argparse
import json
import time
from pathlib import Path
import numpy as np
import rclpy
from rclpy.node import Node
from ackermann_msgs.msg import AckermannDriveStamped
from nav_msgs.msg import Odometry
from std_msgs.msg import String, Float64
from std_srvs.srv import SetBool, Trigger


class Acceptance(Node):
    def __init__(self, carla=False):
        super().__init__('falcon_ros_acceptance')
        self.carla = carla
        self.output, self.status, self.state = None, '', None
        self.samples, self.arrivals = [], []
        self.periods = []
        self.create_subscription(Float64, '/falcon/control_period', lambda m: self.periods.append(m.data * 1000), 100)
        self.pub = self.create_publisher(AckermannDriveStamped, '/falcon/command', 10)
        self.manual_pub = self.create_publisher(AckermannDriveStamped, '/falcon/manual_command', 10)
        self.state_pub = self.create_publisher(Odometry, '/falcon/state', 10)
        self.create_subscription(AckermannDriveStamped, '/falcon/safe_command', self.on_output, 100)
        self.create_subscription(String, '/falcon/safety_status', lambda m: setattr(self, 'status', m.data), 10)
        self.create_subscription(Odometry, '/falcon/state', lambda m: setattr(self, 'state', m), 10)

    def on_output(self, msg):
        self.output = msg
        self.samples.append((self.get_clock().now().nanoseconds-rclpy.time.Time.from_msg(msg.header.stamp).nanoseconds)/1e6)
        self.arrivals.append(time.monotonic())

    def pump(self, seconds, *, state=True, command=True, steer=0.1, accel=1., manual=False):
        deadline = time.monotonic() + seconds
        while time.monotonic() < deadline:
            stamp = self.get_clock().now().to_msg()
            if state and not self.carla:
                odom = Odometry()
                odom.header.stamp = stamp
                odom.header.frame_id = 'map'
                odom.child_frame_id = 'rear_axle'
                odom.pose.pose.orientation.w = 1.
                self.state_pub.publish(odom)
            if command:
                msg = AckermannDriveStamped()
                msg.header.stamp = stamp
                msg.header.frame_id = 'rear_axle'
                msg.drive.steering_angle, msg.drive.acceleration = float(steer), float(accel)
                (self.manual_pub if manual else self.pub).publish(msg)
                if manual:
                    normal = AckermannDriveStamped()
                    normal.header = msg.header
                    normal.drive.steering_angle, normal.drive.acceleration = 0.1, 1.0
                    self.pub.publish(normal)
            rclpy.spin_once(self, timeout_sec=0.01)

    def service(self, typ, name, req):
        client = self.create_client(typ, name)
        assert client.wait_for_service(timeout_sec=10), name
        future = client.call_async(req)
        rclpy.spin_until_future_complete(self, future, timeout_sec=5)
        assert future.done()
        return future.result()

    def run(self):
        self.pump(1., accel=-6.)
        assert self.output is not None
        assert self.service(Trigger, '/falcon/reset_safety', Trigger.Request()).success
        assert not self.service(SetBool, '/falcon/manual_override', SetBool.Request(data=True)).success
        if self.carla:
            initial = (self.state.pose.pose.position.x, self.state.pose.pose.position.y)
            self.pump(3., steer=0., accel=2.)
            final = (self.state.pose.pose.position.x, self.state.pose.pose.position.y)
            displacement = float(np.linalg.norm(np.subtract(final, initial)))
            assert displacement > 0.3, f'CARLA vehicle did not move {displacement}'
            assert self.state.twist.twist.linear.x > 0.3
            self.pump(4., steer=0., accel=-6.)
            assert self.state.twist.twist.linear.x < 0.1
            return {'status': 'PASS_REAL_SIM_ROS', 'platform': 'CARLA 0.9.15 via real DDS Ackermann safety bridge',
                    'displacement_m': displacement, 'stopped_speed_mps': self.state.twist.twist.linear.x,
                    'sample_count': len(self.samples), 'latency_ms': {f'p{p}': float(np.percentile(self.samples,p)) for p in [50,95,99]},
                    'control_period_ms': {f'p{p}': float(np.percentile(self.periods,p)) for p in [50,95,99]},
                    'control_period_jitter_std_ms': float(np.std(self.periods))}
        self.pump(.5)
        assert abs(self.output.drive.steering_angle-.1) < .002 and self.output.drive.acceleration == 1.
        self.pump(.3, command=False)
        assert self.output.drive.acceleration == -6. and 'command_timeout' in self.status
        self.pump(.1)
        assert self.output.drive.acceleration == -6., 'Timeout is latched'
        assert self.service(Trigger, '/falcon/reset_safety', Trigger.Request()).success
        self.pump(.3, steer=5., accel=50.)
        assert abs(self.output.drive.steering_angle) <= .5 and self.output.drive.acceleration == 3.
        assert self.service(SetBool, '/falcon/software_estop', SetBool.Request(data=True)).success
        self.pump(.1)
        assert self.output.drive.acceleration == -6.
        assert not self.service(SetBool, '/falcon/software_estop', SetBool.Request(data=False)).success
        assert self.service(Trigger, '/falcon/reset_safety', Trigger.Request()).success
        self.pump(.1, manual=True, steer=-.05, accel=-1.)
        assert self.service(SetBool, '/falcon/manual_override', SetBool.Request(data=True)).success
        self.pump(.3, manual=True, steer=-.05, accel=-1.)
        assert 'manual' in self.status and abs(self.output.drive.steering_angle+.05)<.002
        assert self.service(SetBool, '/falcon/manual_override', SetBool.Request(data=False)).success
        self.pump(.3, state=False)
        assert self.output.drive.acceleration == -6. and 'state_timeout' in self.status
        assert not self.service(Trigger, '/falcon/reset_safety', Trigger.Request()).success
        intervals = np.diff(self.arrivals)*1000
        return {'status': 'PASS_REAL_ROS_MOCK_IO', 'timestamp_timeout_pass': True, 'latched_stop_pass': True,
                'clamp_rate_limit_pass': True, 'software_estop_pass': True, 'manual_override_pass': True,
                'stale_reset_rejected': True, 'sample_count': len(self.samples),
                'latency_ms': {f'p{p}': float(np.percentile(self.samples,p)) for p in [50,95,99]},
                'interval_ms': {f'p{p}': float(np.percentile(intervals,p)) for p in [50,95,99]},
                'jitter_std_ms': float(np.std(intervals)),
                'control_period_ms': {f'p{p}': float(np.percentile(self.periods,p)) for p in [50,95,99]},
                'control_period_jitter_std_ms': float(np.std(self.periods)),
                'measurement': 'WSL local DDS callback age. Synthetic stationary input, real ROS safety core.'}


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--output', required=True)
    parser.add_argument('--carla', action='store_true')
    args = parser.parse_args()
    rclpy.init()
    node = Acceptance(args.carla)
    try:
        result = node.run()
        Path(args.output).write_text(json.dumps(result, indent=2))
        print(json.dumps(result, indent=2))
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
