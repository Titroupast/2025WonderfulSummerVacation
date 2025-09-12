#!/usr/bin/env python3
import time
from threading import Thread

import rclpy
from rclpy.node import Node
from protocol.msg import TouchStatus


class TouchStandModule(Node):
    """
    触摸站立检测模块（改进版）
    - 只负责触摸检测
    - 检测到双击触摸时，异步调用主程序注册的回调函数触发站立
    """

    def __init__(self, touch_topic='/mi_desktop_48_b0_2d_7b_03_9f/touch_status', stand_callback=None):
        super().__init__('touch_stand_detector')

        self.touch_topic = touch_topic
        self.sub = None  # ROS 订阅器
        self._last_trigger = False
        self.motion_in_progress = False
        self.stand_callback = stand_callback  # 注册回调函数

        self.get_logger().info(f"触摸模块已初始化，触摸 topic: {touch_topic}")

    # ------------------- 对外 API -------------------
    def start_listening(self):
        """开始异步监听触摸"""
        if self.sub is None:
            self.sub = self.create_subscription(
                TouchStatus,
                self.touch_topic,
                self._touch_callback,
                10
            )
            self.get_logger().info("已开始监听触摸事件")

    def wait_for_touch_stand(self, timeout=None):
        """
        阻塞等待双击触摸触发站立
        :param timeout: 超时时间（秒），None 表示一直等待
        """
        self.start_listening()
        self._last_trigger = False
        start_time = time.time()
        while rclpy.ok():
            if not self.motion_in_progress and self._last_trigger:
                self._last_trigger = False
                return True
            if timeout is not None and (time.time() - start_time) > timeout:
                self.get_logger().warn("等待触摸站立超时")
                return False
            rclpy.spin_once(self, timeout_sec=0.05)

    # ------------------- 内部方法 -------------------
    def _touch_callback(self, msg: TouchStatus):
        """触摸回调处理"""
        if getattr(msg, "touch_state", 0) != 0x03:  # 双击触发
            return
        if self.motion_in_progress:
            return

        self.motion_in_progress = True
        self._last_trigger = True

        # 异步调用主程序注册的站立回调
        if self.stand_callback:
            Thread(target=self.stand_callback, daemon=True).start()

        self.motion_in_progress = False
