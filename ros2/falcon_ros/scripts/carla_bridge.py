#!/usr/bin/env python3
"""Real CARLA rear-axle state/safe command transport; owns ticks only with --tick.

Default attaches an existing vehicle without modifying world settings. --spawn
owns/destroys one actor. --tick must only be used when no other client ticks.
"""
import argparse
import math
import time
import carla
import rclpy
from rclpy.node import Node
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from ackermann_msgs.msg import AckermannDriveStamped
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TransformStamped
from tf2_ros import TransformBroadcaster
from hal.carla_adapter import CarlaAdapter
from vehicle_model import VehicleCommand


class CarlaBridge(Node):
    def __init__(self, args):
        super().__init__('falcon_carla_bridge')
        self.client = carla.Client(args.host, args.port)
        self.client.set_timeout(10.)
        self.world = self.client.get_world()
        self.owned = args.spawn
        self.tick_owner = args.tick
        self.original = self.world.get_settings()
        if self.tick_owner:
            settings = self.world.get_settings()
            settings.synchronous_mode = True
            settings.fixed_delta_seconds = 0.05
            settings.no_rendering_mode = True
            self.world.apply_settings(settings)
        if self.owned:
            blueprint = self.world.get_blueprint_library().find('vehicle.tesla.model3')
            self.vehicle = None
            for transform in self.world.get_map().get_spawn_points()[:30]:
                self.vehicle = self.world.try_spawn_actor(blueprint, transform)
                if self.vehicle is not None:
                    break
            if self.vehicle is None:
                raise RuntimeError('No collision-free CARLA spawn point')
        else:
            self.vehicle = self.world.get_actor(args.actor_id)
            if self.vehicle is None or not self.vehicle.type_id.startswith('vehicle.'):
                raise ValueError('Valid CARLA vehicle --actor-id required')
        self.adapter = CarlaAdapter(self.client, self.world, self.vehicle)
        self.last_command = time.monotonic()
        self.command_count = 0
        self.max_accel = -6.
        self.pub = self.create_publisher(Odometry, '/falcon/state', 10)
        self.tf = TransformBroadcaster(self)
        self.command_group = MutuallyExclusiveCallbackGroup()
        self.step_group = MutuallyExclusiveCallbackGroup()
        self.create_subscription(AckermannDriveStamped, '/falcon/safe_command', self.command, 10, callback_group=self.command_group)
        self.create_timer(0.05, self.step, callback_group=self.step_group)
        self.get_logger().info(f'REAL_CARLA server={self.client.get_server_version()} map={self.world.get_map().name} actor={self.vehicle.id} tick_owner={self.tick_owner}')

    def command(self, msg):
        age = (self.get_clock().now().nanoseconds - rclpy.time.Time.from_msg(msg.header.stamp).nanoseconds) / 1e9
        if not -0.05 <= age <= 0.2 or msg.header.frame_id != 'rear_axle':
            self.adapter.apply_command(VehicleCommand(0., -6.))
            return
        if all(map(math.isfinite, [msg.drive.steering_angle, msg.drive.acceleration])):
            self.adapter.apply_command(VehicleCommand(msg.drive.steering_angle, msg.drive.acceleration))
            self.last_command = time.monotonic()
            self.command_count += 1
            self.max_accel = max(self.max_accel, msg.drive.acceleration)

    def step(self):
        if time.monotonic() - self.last_command > 0.2:
            self.adapter.apply_command(VehicleCommand(0., -6.))
        if self.tick_owner:
            self.world.tick()
        state = self.adapter.get_state()
        msg = Odometry()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'map'
        msg.child_frame_id = 'rear_axle'
        msg.pose.pose.position.x, msg.pose.pose.position.y = state.x, state.y
        msg.pose.pose.orientation.z = math.sin(state.psi / 2)
        msg.pose.pose.orientation.w = math.cos(state.psi / 2)
        msg.twist.twist.linear.x = state.v
        self.pub.publish(msg)
        tf = TransformStamped(header=msg.header, child_frame_id=msg.child_frame_id)
        tf.transform.translation.x, tf.transform.translation.y = state.x, state.y
        tf.transform.rotation = msg.pose.pose.orientation
        self.tf.sendTransform(tf)

    def close(self):
        self.get_logger().info(f'ROS commands applied={self.command_count} max_acceleration={self.max_accel}')
        self.adapter.apply_command(VehicleCommand(0., -6.))
        if self.owned:
            self.vehicle.destroy()
        if self.tick_owner:
            self.world.apply_settings(self.original)


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--host', default='127.0.0.1')
    parser.add_argument('--port', type=int, default=2000)
    parser.add_argument('--actor-id', type=int, default=-1)
    parser.add_argument('--spawn', action='store_true')
    parser.add_argument('--tick', action='store_true')
    args, ros_args = parser.parse_known_args()
    rclpy.init(args=ros_args)
    node = CarlaBridge(args)
    executor = MultiThreadedExecutor(num_threads=2)
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        executor.shutdown()
        node.close()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
