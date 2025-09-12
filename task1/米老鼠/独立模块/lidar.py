#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan


class LidarObstacleDetector(Node):
    def __init__(self):
        super().__init__('lidar_obstacle_detector')
        self.get_logger().info('初始化激光障碍物检测节点...')

        # 订阅 /scan 话题
        self.subscription = self.create_subscription(
            LaserScan,
            '/mi_desktop_48_b0_2d_7b_03_9f/scan',
            self.scan_callback,
            10
        )
        self.subscription  # 防止被垃圾回收
        self.get_logger().info('订阅 /scan 话题成功。')

    def scan_callback(self, msg: LaserScan):
        # 过滤掉无效数据 (0 或 inf)
        valid_points = [(i, d) for i, d in enumerate(msg.ranges) if d > 0 and d < float('inf')]

        if valid_points:
            # 找到最近障碍物的索引和距离
            min_index, min_distance = min(valid_points, key=lambda x: x[1])

            # 计算方向（弧度）
            angle_rad = msg.angle_min + min_index * msg.angle_increment
            # 转换成角度
            angle_deg = angle_rad * 180.0 / 3.141592653589793

            self.get_logger().info(
                f'最近障碍物距离: {min_distance:.2f} 米，方向: {angle_deg:.1f}°'
            )
        else:
            self.get_logger().warn('没有有效的激光数据！')


def main(args=None):
    rclpy.init(args=args)
    node = LidarObstacleDetector()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('收到中断信号，正在关闭节点...')
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
