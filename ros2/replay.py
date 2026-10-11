#!/usr/bin/env python3
"""Receive recorded safe commands through genuine rosbag DDS replay.

Run after stopping the CARLA/HAL publishers and adapters. This script observes
messages; it never applies old commands to a simulator or physical actuator.
"""
import argparse
import json
import os
import signal
import subprocess
import time
from pathlib import Path
import rclpy
from ackermann_msgs.msg import AckermannDriveStamped


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--bag', required=True)
    parser.add_argument('--output', required=True)
    args = parser.parse_args()
    rclpy.init()
    node = rclpy.create_node('falcon_bag_replay_observer')
    count = [0]
    node.create_subscription(AckermannDriveStamped, '/falcon/safe_command', lambda m: count.__setitem__(0, count[0]+1), 100)
    logfile = open(Path(args.output).with_suffix('.log'), 'w')
    process = subprocess.Popen(['ros2', 'bag', 'play', args.bag, '--rate', '2'], stdout=logfile, stderr=subprocess.STDOUT, start_new_session=True)
    try:
        deadline = time.monotonic()+30
        while process.poll() is None and time.monotonic()<deadline:
            rclpy.spin_once(node, timeout_sec=.05)
        assert count[0]>100, f'No real DDS replay messages {count[0]}'
        result = {'status': 'PASS_REAL_ROSBAG', 'safe_command_messages_received': count[0], 'bag_path': Path(args.bag).name,
                  'mode': 'Passive observer; simulator/hardware adapters stopped before playback'}
        Path(args.output).write_text(json.dumps(result, indent=2))
        print(json.dumps(result, indent=2))
    finally:
        if process.poll() is None:
            os.killpg(process.pid, signal.SIGINT)
            process.wait(timeout=10)
        logfile.close()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
