#!/usr/bin/env python3
import math
import os
import subprocess
import time

import rclpy
from rclpy.node import Node
from tf2_ros import (
    Buffer,
    TransformListener,
    LookupException,
    ConnectivityException,
    ExtrapolationException,
)


class LapManager(Node):
    def __init__(self):
        super().__init__('lap_manager')
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.start_x = None
        self.start_y = None
        self.is_away = False
        self.lap_count = 0

        self.last_tf_warn_time = 0.0

        self.in_home_area_count = 0
        self.required_home_counts = 10

        self.away_dist_threshold = 2.0
        self.home_dist_threshold = 0.5

        self.timer = self.create_timer(0.1, self.check_lap)
        self.get_logger().info('Lap Manager Started. Waiting for odom->base_link TF...')

    def check_lap(self):
        try:
            t = self.tf_buffer.lookup_transform('odom', 'base_link', rclpy.time.Time())
            x = t.transform.translation.x
            y = t.transform.translation.y

            if self.start_x is None:
                self.start_x = x
                self.start_y = y
                self.get_logger().info(f'Start Position Recorded (odom): x={x:.2f}, y={y:.2f}')
                return

            dist = math.sqrt((x - self.start_x) ** 2 + (y - self.start_y) ** 2)

            if not self.is_away:
                if dist > self.away_dist_threshold:
                    self.is_away = True
                    self.get_logger().info(f'Departed! (Dist: {dist:.2f}m)')
            else:
                if dist < self.home_dist_threshold:
                    self.in_home_area_count += 1
                else:
                    self.in_home_area_count = 0

                if self.in_home_area_count >= self.required_home_counts:
                    self.lap_count += 1
                    self.get_logger().info(f'>>> LAP {self.lap_count} COMPLETED! <<<')
                    self.save_map()

                    self.is_away = False
                    self.in_home_area_count = 0

        except (LookupException, ConnectivityException, ExtrapolationException) as e:
            now = time.time()
            if now - self.last_tf_warn_time > 5.0:
                self.get_logger().warn(f'TF wait (odom->base_link): {e}')
                self.last_tf_warn_time = now
        except Exception as e:
            self.get_logger().error(f'Unexpected error: {e}')

    def save_map(self):
        save_dir = os.path.join(os.path.expanduser('~'), 'maps')
        os.makedirs(save_dir, exist_ok=True)

        file_prefix = f"my_race_map_lap{self.lap_count}"
        map_path = os.path.join(save_dir, file_prefix)

        self.get_logger().info(f'Saving map to: {map_path} ...')
        try:
            subprocess.run(['ros2', 'run', 'nav2_map_server', 'map_saver_cli', '-f', map_path], check=True)
        except Exception as e:
            self.get_logger().error(f'Map save failed: {e}')


def main():
    rclpy.init()
    node = LapManager()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        rclpy.shutdown()


if __name__ == '__main__':
    main()
