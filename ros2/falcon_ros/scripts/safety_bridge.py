#!/usr/bin/env python3
"""SI rear-axle Odometry -> SafetySupervisor -> stamped Ackermann output.

Set FALCON_PROJECT_ROOT/PYTHONPATH to the repository. ROS wall stamps are
converted to local monotonic age envelopes before invoking the shared safety
core. This node never emits a fabricated hardware protocol.
"""
import math
import time
import rclpy
from rclpy.node import Node
from ackermann_msgs.msg import AckermannDriveStamped
from nav_msgs.msg import Odometry
from std_msgs.msg import String, Float64
from std_srvs.srv import SetBool, Trigger
from control.safety import SafetySupervisor, SafetyConfig
from vehicle_model import VehicleState, VehicleCommand


class SafetyBridge(Node):
    def __init__(self):
        super().__init__('falcon_safety_bridge')
        defaults = SafetyConfig()
        parameters = {}
        for name, value in vars(defaults).items():
            configured = self.declare_parameter(name, list(value) if name == 'geofence' else value).value
            parameters[name] = tuple(configured) if name == 'geofence' else configured
        self.supervisor = SafetySupervisor(SafetyConfig(**parameters))
        self.state = VehicleState(0., 0., 0., 0.)
        self.command = VehicleCommand(0., 0.)
        self.state_time = self.command_time = -math.inf
        self.have_state = self.have_command = False
        self.estop = False
        self.manual = False
        self.manual_command = VehicleCommand(0., 0.)
        self.manual_time = -math.inf
        self.output = self.create_publisher(AckermannDriveStamped, '/falcon/safe_command', 10)
        self.status = self.create_publisher(String, '/falcon/safety_status', 10)
        self.period_pub = self.create_publisher(Float64, '/falcon/control_period', 100)
        self.create_subscription(Odometry, '/falcon/state', self.on_state, 10)
        self.create_subscription(AckermannDriveStamped, '/falcon/command', self.on_command, 10)
        self.create_subscription(AckermannDriveStamped, '/falcon/manual_command', self.on_manual, 10)
        self.create_service(SetBool, '/falcon/software_estop', self.stop)
        self.create_service(SetBool, '/falcon/manual_override', self.override)
        self.create_service(Trigger, '/falcon/reset_safety', self.reset)
        self.last_tick = time.monotonic()
        self.create_timer(0.01, self.tick)

    def stamp(self, header):
        age = (self.get_clock().now().nanoseconds - rclpy.time.Time.from_msg(header.stamp).nanoseconds) / 1e9
        return time.monotonic() - age

    def on_state(self, msg):
        p, q = msg.pose.pose.position, msg.pose.pose.orientation
        values = [p.x, p.y, q.x, q.y, q.z, q.w, msg.twist.twist.linear.x]
        if not all(map(math.isfinite, values)) or msg.header.frame_id != 'map' or msg.child_frame_id != 'rear_axle':
            self.estop = True
            return
        yaw = math.atan2(2 * (q.w*q.z + q.x*q.y), 1 - 2*(q.y*q.y + q.z*q.z))
        self.state = VehicleState(p.x, p.y, yaw, msg.twist.twist.linear.x)
        self.state_time = self.stamp(msg.header)
        self.have_state = True

    def on_command(self, msg):
        if not all(map(math.isfinite, [msg.drive.steering_angle, msg.drive.acceleration])) or msg.header.frame_id != 'rear_axle':
            self.estop = True
            return
        self.command = VehicleCommand(msg.drive.steering_angle, msg.drive.acceleration)
        self.command_time = self.stamp(msg.header)
        self.have_command = True

    def on_manual(self, msg):
        if all(map(math.isfinite, [msg.drive.steering_angle, msg.drive.acceleration])) and msg.header.frame_id == 'rear_axle':
            self.manual_command = VehicleCommand(msg.drive.steering_angle, msg.drive.acceleration)
            self.manual_time = self.stamp(msg.header)
        else:
            self.estop = True

    def stop(self, req, res):
        if req.data:
            self.estop = True
        res.success = req.data
        res.message = 'Software stop latched; reset requires fresh stationary state'
        return res

    def override(self, req, res):
        now = time.monotonic()
        if req.data and (now-self.manual_time > self.supervisor.config.max_command_age or self.manual_time > now):
            res.success = False
            res.message = 'Publish fresh manual command before enabling override'
            return res
        self.manual = req.data
        res.success = True
        res.message = 'Manual Ackermann stream requires timestamps and same safety limits'
        return res

    def reset(self, req, res):
        now = time.monotonic()
        if not self.have_state or now-self.state_time > self.supervisor.config.max_state_age or self.state_time > now:
            res.success = False
            res.message = 'Fresh measured state required'
            return res
        try:
            self.supervisor.reset(self.state, explicit=True)
            self.estop = False
            res.success = True
            res.message = 'Stationary reset acknowledged'
        except ValueError as error:
            res.success = False
            res.message = str(error)
        return res

    def tick(self):
        now = time.monotonic()
        dt = max(1e-6, now-self.last_tick)
        self.last_tick = now
        self.period_pub.publish(Float64(data=dt))
        if self.have_state and self.have_command:
            result = self.supervisor.evaluate(self.state, self.command, now=now, state_time=self.state_time,
                command_time=self.manual_time if self.manual else self.command_time, dt=dt,
                emergency_stop=self.estop, manual=self.manual_command if self.manual else None)
            command = result.command
            status = result.mode.value + ':' + ','.join(result.reasons)
        else:
            command = VehicleCommand(0., -self.supervisor.config.max_decel)
            status = 'waiting_for_timestamped_state_and_command'
        output = AckermannDriveStamped()
        output.header.stamp = self.get_clock().now().to_msg()
        output.header.frame_id = 'rear_axle'
        output.drive.steering_angle = float(command.delta)
        output.drive.acceleration = float(command.a)
        self.output.publish(output)
        self.status.publish(String(data=status))


def main():
    rclpy.init()
    node = SafetyBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
