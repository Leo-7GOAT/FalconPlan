#!/usr/bin/env python3
import argparse
import os
import signal
import subprocess
import sys
import time
from pathlib import Path


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--carla', action='store_true')
    parser.add_argument('--host', default='127.0.0.1')
    args = parser.parse_args()
    root = Path(__file__).resolve().parent
    output = root / 'evidence' / ('carla' if args.carla else 'mock_io')
    output.mkdir(parents=True, exist_ok=True)
    env = os.environ.copy()
    env['ROS_DOMAIN_ID'] = '74'
    env['ROS_LOCALHOST_ONLY'] = '1'
    env['PYTHONPATH'] = str(root.parent) + ':' + env.get('PYTHONPATH', '')
    processes, streams = [], []

    def start(command, log):
        stream = open(output / log, 'w')
        streams.append(stream)
        process = subprocess.Popen(command, stdout=stream, stderr=subprocess.STDOUT, env=env, start_new_session=True)
        processes.append(process)
        return process

    def stop(process):
        if process.poll() is None:
            os.killpg(process.pid, signal.SIGINT)
            try:
                process.wait(timeout=10)
            except subprocess.TimeoutExpired:
                os.killpg(process.pid, signal.SIGTERM)
                process.wait(timeout=5)

    try:
        start(['ros2', 'launch', 'falcon_ros', 'bringup.launch.py'], 'launch.log')
        if args.carla:
            start(['ros2', 'run', 'falcon_ros', 'carla_bridge.py', '--host', args.host, '--spawn', '--tick'], 'carla_bridge.log')
        time.sleep(3)
        bag_path = output / ('bag_' + str(time.time_ns()))
        bag = start(['ros2', 'bag', 'record', '-o', str(bag_path), '/falcon/state', '/falcon/command', '/falcon/safe_command', '/falcon/safety_status', '/falcon/control_period', '/tf'], 'bag_record.log')
        command = ['ros2', 'run', 'falcon_ros', 'acceptance.py', '--output', str(output / 'acceptance.json')]
        if args.carla:
            command.append('--carla')
        result = subprocess.run(command, stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True, env=env, timeout=60)
        (output / 'acceptance.log').write_text(result.stdout)
        print(result.stdout)
        assert result.returncode == 0, f'Acceptance exited {result.returncode}'
        stop(bag)
        info = subprocess.run(['ros2', 'bag', 'info', str(bag_path)], capture_output=True, text=True, env=env, check=True)
        (output / 'bag_info.txt').write_text(info.stdout)
        for process in reversed(processes):
            stop(process)
        subprocess.run([sys.executable, str(root / 'replay.py'), '--bag', str(bag_path), '--output', str(output / 'replay.json')], env=env, check=True, timeout=40)
    finally:
        for process in reversed(processes):
            stop(process)
        for stream in streams:
            stream.close()


if __name__ == '__main__':
    main()
