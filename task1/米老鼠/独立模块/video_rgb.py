#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import os
from datetime import datetime

class SimpleVideoRecorder(Node):
    def __init__(self):
        super().__init__('simple_video_recorder')
        self.bridge = CvBridge()

        # 保存目录
        self.save_dir = os.path.join(os.getcwd(), "videos_auto")
        os.makedirs(self.save_dir, exist_ok=True)
        self.get_logger().info(f"保存目录: {self.save_dir}")

        # 图像话题
        self.topic_name = '/image_rgb'

        # 视频文件路径
        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        safe_name = self.topic_name.replace('/', '_').strip('_')
        self.video_path = os.path.join(self.save_dir, f"{safe_name}_{timestamp}.mp4")

        self.video_writer = None
        self.frame_size = None
        self.fps = 20.0  # 帧率，可以根据你的实际话题频率调整

        # 订阅
        self.sub = self.create_subscription(
            Image,
            self.topic_name,
            self.image_callback,
            10
        )

        self.get_logger().info("🎥 开始录像，按 Ctrl+C 停止并保存")

    def image_callback(self, msg):
        try:
            frame = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        except Exception as e:
            self.get_logger().error(f"图像转换失败: {e}")
            return

        # 初始化 VideoWriter
        if self.video_writer is None:
            height, width = frame.shape[:2]
            self.frame_size = (width, height)
            fourcc = cv2.VideoWriter_fourcc(*'mp4v')  # mp4 编码
            self.video_writer = cv2.VideoWriter(self.video_path, fourcc, self.fps, self.frame_size)
            self.get_logger().info(f"📹 视频文件: {self.video_path}")

        self.video_writer.write(frame)  # 写入一帧

    def destroy_node(self):
        # 关闭 VideoWriter
        if self.video_writer:
            self.video_writer.release()
            self.get_logger().info(f"💾 视频已保存到: {self.video_path}")
        super().destroy_node()

def main():
    rclpy.init()
    node = SimpleVideoRecorder()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()

